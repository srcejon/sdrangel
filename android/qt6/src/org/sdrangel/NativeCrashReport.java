///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
///////////////////////////////////////////////////////////////////////////////////

package org.sdrangel;

import android.annotation.SuppressLint;
import android.app.ActivityManager;
import android.app.ApplicationExitInfo;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageInfo;
import android.os.Build;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.TimeZone;

/** Reads the newest unacknowledged native tombstone made available by Android. */
final class NativeCrashReport
{
    private static final String PREFERENCES_NAME = "native_crash_reports";
    private static final String ACKNOWLEDGED_REPORT_ID = "acknowledged_report_id";
    private static final int EXIT_HISTORY_LIMIT = 16;
    private static final int MAX_TRACE_BYTES = 8 * 1024 * 1024;
    private static final int MAX_BACKTRACE_FRAMES = 128;

    static final class Result
    {
        final String m_id;
        final String m_text;

        Result(String id, String text)
        {
            m_id = id;
            m_text = text;
        }
    }

    private static final class TraceData
    {
        byte[] m_data;
        boolean m_truncated;
    }

    private static final class SignalInfo
    {
        int m_number;
        int m_code;
        String m_name = "";
        String m_codeName = "";
        boolean m_hasFaultAddress;
        long m_faultAddress;
    }

    private static final class BacktraceFrame
    {
        long m_relativePc;
        long m_pc;
        String m_functionName = "";
        long m_functionOffset;
        String m_fileName = "";
        String m_buildId = "";
    }

    private static final class ThreadInfo
    {
        long m_mapId = -1;
        int m_id;
        String m_name = "";
        final ArrayList<String> m_notes = new ArrayList<String>();
        final ArrayList<BacktraceFrame> m_frames = new ArrayList<BacktraceFrame>();
    }

    private static final class Tombstone
    {
        int m_architecture = -1;
        long m_pid;
        long m_tid;
        long m_processUptimeSeconds;
        String m_buildFingerprint = "";
        String m_timestamp = "";
        String m_abortMessage = "";
        String m_parseWarning = "";
        final ArrayList<String> m_commandLine = new ArrayList<String>();
        final ArrayList<String> m_causes = new ArrayList<String>();
        final ArrayList<ThreadInfo> m_threads = new ArrayList<ThreadInfo>();
        SignalInfo m_signal;
    }

    /** Small protobuf wire reader for the public Android tombstone schema. */
    private static final class ProtoReader
    {
        private final byte[] m_data;
        private int m_position;
        private final int m_end;

        ProtoReader(byte[] data)
        {
            this(data, 0, data.length);
        }

        ProtoReader(byte[] data, int position, int end)
        {
            m_data = data;
            m_position = position;
            m_end = end;
        }

        boolean hasRemaining()
        {
            return m_position < m_end;
        }

        long readVarint() throws IOException
        {
            long value = 0;
            for (int shift = 0; shift < 64; shift += 7)
            {
                if (m_position >= m_end) {
                    throw new IOException("Truncated protobuf varint");
                }
                int next = m_data[m_position++] & 0xff;
                value |= (long) (next & 0x7f) << shift;
                if ((next & 0x80) == 0) {
                    return value;
                }
            }
            throw new IOException("Invalid protobuf varint");
        }

        ProtoReader readMessage() throws IOException
        {
            long lengthValue = readVarint();
            if ((lengthValue < 0) || (lengthValue > Integer.MAX_VALUE)) {
                throw new IOException("Invalid protobuf message length");
            }
            int length = (int) lengthValue;
            if (length > m_end - m_position) {
                throw new IOException("Truncated protobuf message");
            }
            ProtoReader result = new ProtoReader(m_data, m_position, m_position + length);
            m_position += length;
            return result;
        }

        String readString() throws IOException
        {
            ProtoReader value = readMessage();
            return new String(value.m_data, value.m_position, value.m_end - value.m_position,
                    StandardCharsets.UTF_8);
        }

        void skipField(int wireType) throws IOException
        {
            switch (wireType)
            {
            case 0:
                readVarint();
                break;
            case 1:
                skipBytes(8);
                break;
            case 2:
                long lengthValue = readVarint();
                if ((lengthValue < 0) || (lengthValue > Integer.MAX_VALUE)) {
                    throw new IOException("Invalid protobuf field length");
                }
                skipBytes((int) lengthValue);
                break;
            case 5:
                skipBytes(4);
                break;
            default:
                throw new IOException("Unsupported protobuf wire type " + wireType);
            }
        }

        private void skipBytes(int count) throws IOException
        {
            if ((count < 0) || (count > m_end - m_position)) {
                throw new IOException("Truncated protobuf field");
            }
            m_position += count;
        }
    }

    private NativeCrashReport()
    {
    }

    @SuppressLint("NewApi")
    static Result load(Context context)
    {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) {
            return null;
        }

        try
        {
            ActivityManager manager = (ActivityManager) context.getSystemService(Context.ACTIVITY_SERVICE);
            if (manager == null) {
                return null;
            }

            List<ApplicationExitInfo> history = manager.getHistoricalProcessExitReasons(
                    context.getPackageName(), 0, EXIT_HISTORY_LIMIT);
            ApplicationExitInfo newest = null;
            for (ApplicationExitInfo exit : history)
            {
                if (exit.getReason() != ApplicationExitInfo.REASON_CRASH_NATIVE) {
                    continue;
                }
                String processName = exit.getProcessName();
                if ((processName != null) && !processName.equals(context.getPackageName())
                        && !processName.startsWith(context.getPackageName() + ":")) {
                    continue;
                }
                if ((newest == null) || (exit.getTimestamp() > newest.getTimestamp())) {
                    newest = exit;
                }
            }
            if (newest == null) {
                return null;
            }

            String reportId = makeReportId(newest);
            SharedPreferences preferences = context.getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE);
            if (reportId.equals(preferences.getString(ACKNOWLEDGED_REPORT_ID, ""))) {
                return null;
            }

            TraceData traceData = readTrace(newest.getTraceInputStream());
            Tombstone tombstone = null;
            if ((traceData != null) && (traceData.m_data.length != 0)) {
                tombstone = parseTombstone(traceData.m_data);
            }
            return new Result(reportId, formatReport(context, newest, traceData, tombstone));
        }
        catch (Exception exception)
        {
            android.util.Log.w("sdrangel", "Cannot read previous native crash report", exception);
            return null;
        }
    }

    static void acknowledge(Context context, String reportId)
    {
        if ((reportId == null) || reportId.isEmpty()) {
            return;
        }
        context.getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE)
                .edit().putString(ACKNOWLEDGED_REPORT_ID, reportId).apply();
    }

    private static String makeReportId(ApplicationExitInfo exit)
    {
        return exit.getTimestamp() + ":" + exit.getPid() + ":" + exit.getStatus() + ":"
                + String.valueOf(exit.getProcessName());
    }

    private static TraceData readTrace(InputStream input) throws IOException
    {
        if (input == null) {
            return null;
        }

        try (InputStream source = input;
             ByteArrayOutputStream output = new ByteArrayOutputStream(64 * 1024))
        {
            byte[] buffer = new byte[32 * 1024];
            int remaining = MAX_TRACE_BYTES;
            while (remaining > 0)
            {
                int count = source.read(buffer, 0, Math.min(buffer.length, remaining));
                if (count < 0) {
                    break;
                }
                if (count == 0) {
                    continue;
                }
                output.write(buffer, 0, count);
                remaining -= count;
            }

            TraceData result = new TraceData();
            result.m_data = output.toByteArray();
            result.m_truncated = (remaining == 0) && (source.read() >= 0);
            return result;
        }
    }

    private static String formatReport(Context context, ApplicationExitInfo exit,
                                       TraceData traceData, Tombstone tombstone)
    {
        StringBuilder report = new StringBuilder(8192);
        report.append("SDRangel previous native crash\n\n");
        appendLine(report, "Application", applicationVersion(context));
        appendLine(report, "Android", Build.VERSION.RELEASE + " (API " + Build.VERSION.SDK_INT + ")");
        appendLine(report, "Device", Build.MANUFACTURER + " " + Build.MODEL);
        appendLine(report, "ABI", join(Build.SUPPORTED_ABIS, ", "));
        appendLine(report, "Crash time (local)", formatDate(exit.getTimestamp(), false));
        appendLine(report, "Crash time (UTC)", formatDate(exit.getTimestamp(), true));
        appendLine(report, "Process", String.valueOf(exit.getProcessName()));
        appendLine(report, "PID", Integer.toString(exit.getPid()));
        appendLine(report, "Signal/status", Integer.toString(exit.getStatus()));
        appendLine(report, "Description", nullToEmpty(exit.getDescription()));
        appendLine(report, "Last PSS", exit.getPss() + " KiB");
        appendLine(report, "Last RSS", exit.getRss() + " KiB");

        if (traceData == null)
        {
            report.append("\nAndroid no longer has a tombstone trace for this crash.\n");
            return report.toString();
        }

        report.append("\nNative tombstone\n");
        appendLine(report, "Trace bytes", Integer.toString(traceData.m_data.length)
                + (traceData.m_truncated ? " (truncated)" : ""));
        if (tombstone == null)
        {
            report.append("The tombstone could not be decoded.\n");
            return report.toString();
        }

        appendLine(report, "Architecture", architectureName(tombstone.m_architecture));
        appendLine(report, "Build fingerprint", tombstone.m_buildFingerprint);
        appendLine(report, "Tombstone time", tombstone.m_timestamp);
        appendLine(report, "Process uptime", tombstone.m_processUptimeSeconds + " s");
        if (!tombstone.m_commandLine.isEmpty()) {
            appendLine(report, "Command line", join(tombstone.m_commandLine.toArray(new String[0]), " "));
        }
        if (tombstone.m_signal != null)
        {
            SignalInfo signal = tombstone.m_signal;
            String signalText = signal.m_number + (signal.m_name.isEmpty() ? "" : " (" + signal.m_name + ")");
            if ((signal.m_code != 0) || !signal.m_codeName.isEmpty()) {
                signalText += ", code " + signal.m_code
                        + (signal.m_codeName.isEmpty() ? "" : " (" + signal.m_codeName + ")");
            }
            appendLine(report, "Signal", signalText);
            if (signal.m_hasFaultAddress) {
                appendLine(report, "Fault address", hex(signal.m_faultAddress));
            }
        }
        appendLine(report, "Abort message", tombstone.m_abortMessage);
        for (String cause : tombstone.m_causes) {
            appendLine(report, "Cause", cause);
        }
        if (!tombstone.m_parseWarning.isEmpty()) {
            appendLine(report, "Trace warning", tombstone.m_parseWarning);
        }

        ThreadInfo crashingThread = findCrashingThread(tombstone);
        if (crashingThread == null)
        {
            report.append("\nNo crashing-thread backtrace was present.\n");
            return report.toString();
        }

        report.append("\nCrashing thread ").append(crashingThread.m_id);
        if (!crashingThread.m_name.isEmpty()) {
            report.append(" (").append(crashingThread.m_name).append(')');
        }
        report.append("\n");
        for (String note : crashingThread.m_notes) {
            report.append("  ").append(note).append("\n");
        }
        int frameCount = Math.min(crashingThread.m_frames.size(), MAX_BACKTRACE_FRAMES);
        for (int index = 0; index < frameCount; ++index) {
            appendFrame(report, index, crashingThread.m_frames.get(index));
        }
        if (crashingThread.m_frames.size() > frameCount) {
            report.append("  ... ").append(crashingThread.m_frames.size() - frameCount)
                    .append(" additional frames omitted\n");
        }
        return report.toString();
    }

    @SuppressWarnings("deprecation")
    private static String applicationVersion(Context context)
    {
        try
        {
            PackageInfo info = context.getPackageManager().getPackageInfo(context.getPackageName(), 0);
            long code = Build.VERSION.SDK_INT >= Build.VERSION_CODES.P
                    ? info.getLongVersionCode() : info.versionCode;
            return info.versionName + " (" + code + ")";
        }
        catch (Exception exception)
        {
            return context.getPackageName();
        }
    }

    private static String formatDate(long timestamp, boolean utc)
    {
        SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS z", Locale.US);
        if (utc) {
            format.setTimeZone(TimeZone.getTimeZone("UTC"));
        }
        return format.format(new Date(timestamp));
    }

    private static void appendLine(StringBuilder report, String name, String value)
    {
        if ((value != null) && !value.isEmpty()) {
            report.append(name).append(": ").append(value).append('\n');
        }
    }

    private static void appendFrame(StringBuilder report, int index, BacktraceFrame frame)
    {
        report.append(String.format(Locale.US, "#%02d pc %016x  ", index, frame.m_relativePc));
        report.append(frame.m_fileName.isEmpty() ? "<unknown>" : frame.m_fileName);
        if (!frame.m_functionName.isEmpty())
        {
            report.append(" (").append(frame.m_functionName);
            if (frame.m_functionOffset != 0) {
                report.append('+').append(hex(frame.m_functionOffset));
            }
            report.append(')');
        }
        if (!frame.m_buildId.isEmpty()) {
            report.append(" (BuildId: ").append(frame.m_buildId).append(')');
        }
        if ((frame.m_relativePc == 0) && (frame.m_pc != 0)) {
            report.append(" [pc ").append(hex(frame.m_pc)).append(']');
        }
        report.append('\n');
    }

    private static ThreadInfo findCrashingThread(Tombstone tombstone)
    {
        for (ThreadInfo thread : tombstone.m_threads) {
            if ((thread.m_mapId == tombstone.m_tid) || (thread.m_id == tombstone.m_tid)) {
                return thread;
            }
        }
        return tombstone.m_threads.isEmpty() ? null : tombstone.m_threads.get(0);
    }

    private static String architectureName(int architecture)
    {
        switch (architecture)
        {
        case 0: return "ARM32";
        case 1: return "ARM64";
        case 2: return "x86";
        case 3: return "x86_64";
        case 4: return "RISC-V 64";
        case 5: return "None";
        default: return architecture < 0 ? "" : Integer.toString(architecture);
        }
    }

    private static String nullToEmpty(String value)
    {
        return value == null ? "" : value;
    }

    private static String hex(long value)
    {
        return String.format(Locale.US, "0x%x", value);
    }

    private static String join(String[] values, String separator)
    {
        StringBuilder joined = new StringBuilder();
        for (String value : values)
        {
            if (joined.length() != 0) {
                joined.append(separator);
            }
            joined.append(value);
        }
        return joined.toString();
    }

    private static Tombstone parseTombstone(byte[] data)
    {
        Tombstone tombstone = new Tombstone();
        ProtoReader reader = new ProtoReader(data);
        try
        {
            while (reader.hasRemaining())
            {
                long key = reader.readVarint();
                int field = (int) (key >>> 3);
                int wire = (int) (key & 7);
                switch (field)
                {
                case 1:
                    if (wire == 0) tombstone.m_architecture = (int) reader.readVarint(); else reader.skipField(wire);
                    break;
                case 2:
                    if (wire == 2) tombstone.m_buildFingerprint = reader.readString(); else reader.skipField(wire);
                    break;
                case 4:
                    if (wire == 2) tombstone.m_timestamp = reader.readString(); else reader.skipField(wire);
                    break;
                case 5:
                    if (wire == 0) tombstone.m_pid = reader.readVarint(); else reader.skipField(wire);
                    break;
                case 6:
                    if (wire == 0) tombstone.m_tid = reader.readVarint(); else reader.skipField(wire);
                    break;
                case 9:
                    if (wire == 2) tombstone.m_commandLine.add(reader.readString()); else reader.skipField(wire);
                    break;
                case 10:
                    if (wire == 2) tombstone.m_signal = parseSignal(reader.readMessage()); else reader.skipField(wire);
                    break;
                case 14:
                    if (wire == 2) tombstone.m_abortMessage = reader.readString(); else reader.skipField(wire);
                    break;
                case 15:
                    if (wire == 2) {
                        String cause = parseCause(reader.readMessage());
                        if (!cause.isEmpty()) tombstone.m_causes.add(cause);
                    } else reader.skipField(wire);
                    break;
                case 16:
                    if (wire == 2) {
                        ThreadInfo thread = parseThreadMapEntry(reader.readMessage());
                        if (thread != null) tombstone.m_threads.add(thread);
                    } else reader.skipField(wire);
                    break;
                case 20:
                    if (wire == 0) tombstone.m_processUptimeSeconds = reader.readVarint(); else reader.skipField(wire);
                    break;
                default:
                    reader.skipField(wire);
                    break;
                }
            }
        }
        catch (IOException exception)
        {
            tombstone.m_parseWarning = exception.getMessage();
        }
        return tombstone;
    }

    private static SignalInfo parseSignal(ProtoReader reader) throws IOException
    {
        SignalInfo signal = new SignalInfo();
        while (reader.hasRemaining())
        {
            long key = reader.readVarint();
            int field = (int) (key >>> 3);
            int wire = (int) (key & 7);
            if ((field == 1) && (wire == 0)) signal.m_number = (int) reader.readVarint();
            else if ((field == 2) && (wire == 2)) signal.m_name = reader.readString();
            else if ((field == 3) && (wire == 0)) signal.m_code = (int) reader.readVarint();
            else if ((field == 4) && (wire == 2)) signal.m_codeName = reader.readString();
            else if ((field == 8) && (wire == 0)) signal.m_hasFaultAddress = reader.readVarint() != 0;
            else if ((field == 9) && (wire == 0)) signal.m_faultAddress = reader.readVarint();
            else reader.skipField(wire);
        }
        return signal;
    }

    private static String parseCause(ProtoReader reader) throws IOException
    {
        String result = "";
        while (reader.hasRemaining())
        {
            long key = reader.readVarint();
            int field = (int) (key >>> 3);
            int wire = (int) (key & 7);
            if ((field == 1) && (wire == 2)) result = reader.readString();
            else reader.skipField(wire);
        }
        return result;
    }

    private static ThreadInfo parseThreadMapEntry(ProtoReader reader) throws IOException
    {
        ThreadInfo thread = null;
        long mapId = -1;
        while (reader.hasRemaining())
        {
            long key = reader.readVarint();
            int field = (int) (key >>> 3);
            int wire = (int) (key & 7);
            if ((field == 1) && (wire == 0)) mapId = reader.readVarint();
            else if ((field == 2) && (wire == 2)) thread = parseThread(reader.readMessage());
            else reader.skipField(wire);
        }
        if (thread != null) {
            thread.m_mapId = mapId;
        }
        return thread;
    }

    private static ThreadInfo parseThread(ProtoReader reader) throws IOException
    {
        ThreadInfo thread = new ThreadInfo();
        while (reader.hasRemaining())
        {
            long key = reader.readVarint();
            int field = (int) (key >>> 3);
            int wire = (int) (key & 7);
            if ((field == 1) && (wire == 0)) thread.m_id = (int) reader.readVarint();
            else if ((field == 2) && (wire == 2)) thread.m_name = reader.readString();
            else if ((field == 4) && (wire == 2)) thread.m_frames.add(parseFrame(reader.readMessage()));
            else if ((field == 7) && (wire == 2)) thread.m_notes.add(reader.readString());
            else reader.skipField(wire);
        }
        return thread;
    }

    private static BacktraceFrame parseFrame(ProtoReader reader) throws IOException
    {
        BacktraceFrame frame = new BacktraceFrame();
        while (reader.hasRemaining())
        {
            long key = reader.readVarint();
            int field = (int) (key >>> 3);
            int wire = (int) (key & 7);
            if ((field == 1) && (wire == 0)) frame.m_relativePc = reader.readVarint();
            else if ((field == 2) && (wire == 0)) frame.m_pc = reader.readVarint();
            else if ((field == 4) && (wire == 2)) frame.m_functionName = reader.readString();
            else if ((field == 5) && (wire == 0)) frame.m_functionOffset = reader.readVarint();
            else if ((field == 6) && (wire == 2)) frame.m_fileName = reader.readString();
            else if ((field == 8) && (wire == 2)) frame.m_buildId = reader.readString();
            else reader.skipField(wire);
        }
        return frame;
    }
}
