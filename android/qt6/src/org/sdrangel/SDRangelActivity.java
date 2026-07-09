///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2022 Jon Beniston, M7RCE                                        //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

package org.sdrangel;

import org.qtproject.qt.android.bindings.QtActivity;

import android.hardware.usb.*;
import android.app.*;
import android.content.*;
import android.os.*;
import android.os.PowerManager.WakeLock;
import android.view.WindowManager;
import android.util.Log;

import java.util.HashMap;
import java.util.Vector;
import java.util.Iterator;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class SDRangelActivity extends QtActivity
{
    private String TAG = "sdrangel";
    
    private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";
    private static final long USB_PERMISSION_TIMEOUT_SECONDS = 30;
    private PendingIntent m_permissionIntent;
    private UsbManager m_manager;
    private HandlerThread m_usbReceiverThread;
    private Handler m_usbReceiverHandler;

    private HashMap<Integer, UsbDeviceConnection> m_connectedDevices;  // fd to connection
    private HashMap<String, UsbDevice> m_devicesBySerial;

    private final Object m_lock = new Object();
    
    CountDownLatch m_doneSignal;
    boolean m_permissionGranted;
    int m_fd;

    private WakeLock m_wakeLock;
    
    public SDRangelActivity()
    {
        super();
        Log.e(TAG, "SDRangelActivity");
        m_connectedDevices = new HashMap<Integer, UsbDeviceConnection>();
        m_devicesBySerial = new HashMap<String, UsbDevice>();
    }
    
    @Override
    public void onCreate(Bundle savedInstanceState) 
    {
        super.onCreate(savedInstanceState);
        try {
            m_manager = (UsbManager) getSystemService(Context.USB_SERVICE);
            Intent permissionIntent = new Intent(ACTION_USB_PERMISSION);
            permissionIntent.setPackage(getPackageName());
            m_permissionIntent = PendingIntent.getBroadcast(this, 0, permissionIntent, PendingIntent.FLAG_MUTABLE);
            m_usbReceiverThread = new HandlerThread("SDRangelUsbReceiver");
            m_usbReceiverThread.start();
            m_usbReceiverHandler = new Handler(m_usbReceiverThread.getLooper());

            IntentFilter permissionFilter = new IntentFilter(ACTION_USB_PERMISSION);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                registerReceiver(usbManagerBroadcastReceiver, permissionFilter, null, m_usbReceiverHandler, Context.RECEIVER_NOT_EXPORTED);
            } else {
                registerReceiver(usbManagerBroadcastReceiver, permissionFilter, null, m_usbReceiverHandler);
            }
        } catch (Exception e) {
             Log.e(TAG, "SDRangelActivity::onCreate: Exception " + e.toString());
        }        
    }

    @Override
    public void onDestroy()
    {
        try {
            unregisterReceiver(usbManagerBroadcastReceiver);
        } catch (Exception e) {
            Log.d(TAG, "SDRangelActivity::onDestroy: unregisterReceiver: " + e.toString());
        }
        if (m_usbReceiverThread != null) {
            m_usbReceiverThread.quitSafely();
            m_usbReceiverThread = null;
            m_usbReceiverHandler = null;
        }
        super.onDestroy();
    }

    // Prevent app from being put too sleep (which can stop acquisition)
    public void acquireWakeLock()
    {
        if (m_wakeLock == null)
        {
            PowerManager powerManager = (PowerManager)getSystemService(POWER_SERVICE);
            m_wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "SDRangel::WakeLockTag");
        }
        Log.e(TAG, "Acquiring wake lock");
        m_wakeLock.acquire();
    }
    
    public void releaseWakeLock()
    {
        if (m_wakeLock != null)
        {
            Log.e(TAG, "Releasing wake lock");
            m_wakeLock.release();
        }
    }
    
    public void acquireScreenLock()
    {
        Log.e(TAG, "Acquiring screen lock");
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            }
        });
        
    }
    
    public void releaseScreenLock()
    {
        Log.e(TAG, "Releasing screen lock");
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            }
        });
    }
    
    public String[] listUSBDeviceSerials(int vid, int pid)
    {
        Vector<String> serials = new Vector<String>();
        try 
        {
            synchronized(m_lock) 
            {                                     
                HashMap<String, UsbDevice> deviceList = m_manager.getDeviceList();
                Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
        
                while(deviceIterator.hasNext())
                {
                    UsbDevice device = deviceIterator.next();                    
                    if ((device.getVendorId() == vid) && (device.getProductId() == pid))
                    {
                        // From Android Q, we need permission to read serial
                        if (requestUSBPermission(device))
                        {
                            String serial = device.getSerialNumber();
                            if (serial != null)
                            {
                                Log.d(TAG, "Found a device: ID=" + device.getDeviceId() + " Name=" + device.getDeviceName() + " Serial=" + serial);
                                m_devicesBySerial.put(serial, device);
                                serials.add(serial);
                            }
                            else
                            {
                                Log.d(TAG, "Found a device with no serial: ID=" + device.getDeviceId() + " Name=" + device.getDeviceName());
                            }
                        }
                   }
                }
            }
        }
        catch (Exception e)
        {
            Log.e(TAG, "SDRangelActivity::listUSBDeviceSerials: Exception " + e.toString());
        }        
        return serials.toArray(new String[serials.size()]);
    }

    public int openUSBDevice(String serial)
    {
        Log.e(TAG, "SDRangelActivity::openUSBDevice serial=" + serial);
        try 
        {
            synchronized(m_lock) 
            {                                     
                m_fd = -1;

                UsbDevice device = m_devicesBySerial.get(serial);
                if (device != null)
                {
                    if (requestUSBPermission(device))
                    {
                        String deviceSerial = device.getSerialNumber();
                        if ((deviceSerial != null) && (deviceSerial.compareToIgnoreCase(serial) == 0))
                        {
                            m_fd = connectToUSBDevice(device);
                            return m_fd;
                        }
                    }
                }

                HashMap<String, UsbDevice> deviceList = m_manager.getDeviceList();
                Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();

                while(deviceIterator.hasNext())
                {
                    device = deviceIterator.next();
                    if (m_manager.hasPermission(device))
                    {
                        String deviceSerial = device.getSerialNumber();
                        if ((deviceSerial != null) && (deviceSerial.compareToIgnoreCase(serial) == 0))
                        {
                            m_fd = connectToUSBDevice(device);
                            break;
                        }
                    }
                }
                
                return m_fd;
            }
        } 
        catch (Exception e)
        {
            Log.e(TAG, "SDRangelActivity::openUSBDevice: Exception " + e.toString());
            return -1;
        }
    }
    
    public void closeUSBDevice(int fd)
    {
        Log.e(TAG, "SDRangelActivity::closeUSBDevice: fd=" + fd);
        UsbDeviceConnection connection = m_connectedDevices.get(fd);
        if (connection != null)
        {
            connection.close();
            m_connectedDevices.remove(fd);
        }
    }

    private boolean requestUSBPermission(UsbDevice device) throws InterruptedException
    {
        if (m_manager == null)
        {
            Log.e(TAG, "SDRangelActivity::requestUSBPermission: UsbManager is not available");
            return false;
        }
        if (m_manager.hasPermission(device)) {
            return true;
        }
        if (m_permissionIntent == null)
        {
            Log.e(TAG, "SDRangelActivity::requestUSBPermission: permission intent is not available");
            return false;
        }

        m_permissionGranted = false;
        m_doneSignal = new CountDownLatch(1);
        m_manager.requestPermission(device, m_permissionIntent);
        if (!m_doneSignal.await(USB_PERMISSION_TIMEOUT_SECONDS, TimeUnit.SECONDS))
        {
            Log.e(TAG, "SDRangelActivity::requestUSBPermission: timed out waiting for permission for device " + device);
            return false;
        }

        return m_permissionGranted;
    }

    private int connectToUSBDevice(UsbDevice device)
    {
        UsbDeviceConnection connection = m_manager.openDevice(device);
        if (connection != null)
        {
            // Can disconnect kernel driver with:
            //connection.claimInterface(device.getInterface(0), true);    
            // However, this results in airspyhf.c:libusb_set_configuration returning BUSY

            int fd = connection.getFileDescriptor();
            if (fd >= 0)
            {
                m_connectedDevices.put(fd, connection);
                return fd;
            }
            connection.close();
        }
        return -1;
    }

    private final BroadcastReceiver usbManagerBroadcastReceiver = new BroadcastReceiver()
    {
        public void onReceive(Context context, Intent intent)
        {
            try
            {
                String action = intent.getAction();
                Log.d(TAG, "INTENT ACTION: " + action);
                if (ACTION_USB_PERMISSION.equals(action))
                {
                    synchronized (this)
                    {
                        UsbDevice device = (UsbDevice)intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                        if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
                        {
                            if(device != null)
                            {
                                m_permissionGranted = true;
                                Log.d(TAG, "Granted permission for device " + device.getDeviceId());
                            }
                        }
                        else
                        {
                            Log.d(TAG, "Permission denied for device " + device);
                        }
                        if (m_doneSignal != null) {
                            m_doneSignal.countDown();
                        }
                    }
                }               
            }
            catch(Exception e)
            {
                Log.d(TAG, "Exception: " + e);
            }
        }
    };

}
