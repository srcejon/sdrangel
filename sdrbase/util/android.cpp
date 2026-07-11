///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2022-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
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

#ifdef ANDROID

#include <QDebug>
#include <QFile>

#include <atomic>

#include <android/log.h>

#include "android.h"

namespace {

int nextDocumentTreeRequestCode()
{
    static std::atomic<int> requestCode {2301};
    return requestCode.fetch_add(1, std::memory_order_relaxed);
}

} // namespace

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))

#include <QtCore/private/qandroidextras_p.h>
#include <QJniObject>
#include <QJniEnvironment>

namespace {

QJniObject androidActivity()
{
    return QJniObject::callStaticObjectMethod(
        "org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
}

QJniObject uriFromString(const QString& value)
{
    const QJniObject string = QJniObject::fromString(value);
    return QJniObject::callStaticObjectMethod(
        "android/net/Uri", "parse", "(Ljava/lang/String;)Landroid/net/Uri;", string.object<jstring>());
}

} // namespace

void Android::sendIntent()
{
    QJniObject url = QJniObject::fromString("iqsrc://-f 1090000000 -p 1234 -s 2000000 -a 127.0.0.1 -g 100");
    QJniObject intent = QJniObject::callStaticObjectMethod("android/content/Intent", "parseUri", "(Ljava/lang/String;I)Landroid/content/Intent;", url.object<jstring>(), 0x00000001);   // Creates Intent(ACTION_VIEW, url)
    QtAndroidPrivate::startActivity(intent, 0, [](int requestCode, int resultCode, const QJniObject &data) {
        (void) data;

        qDebug() << "MainCore::sendIntent " << requestCode << resultCode;
    });
}

QStringList Android::listUSBDeviceSerials(int vid, int pid)
{
    QStringList serials;
    QJniEnvironment jniEnv;

    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid())
    {
        QJniObject serialsObj = activity.callObjectMethod("listUSBDeviceSerials", "(II)[Ljava/lang/String;", vid, pid);
        int serialsLen = jniEnv->GetArrayLength(serialsObj.object<jarray>());
        for (int i = 0; i < serialsLen; i++)
        {
            QJniObject arrayElement = jniEnv->GetObjectArrayElement(serialsObj.object<jobjectArray>(), i);
            QString serial = arrayElement.toString();
            serials.append(serial);
        }
    }

    return serials;
}

int Android::openUSBDevice(const QString &serial)
{
    int fd = -1;
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid())
    {
        QJniObject serialsObj = QJniObject::fromString(serial);
        fd = activity.callMethod<jint>("openUSBDevice", "(Ljava/lang/String;)I", serialsObj.object<jstring>());
    }

    return fd;
}

void Android::closeUSBDevice(int fd)
{
    if (fd >= 0)
    {
        QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
        if (activity.isValid()) {
            activity.callMethod<void>("closeUSBDevice", "(I)V", fd);
        } else {
            qCritical() << "MainCore::closeUSBDevice: activity is not valid.";
        }
    }
}

void Android::moveTaskToBack()
{
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<jboolean>("moveTaskToBack", "(Z)Z", true);
    }
}

void Android::acquireWakeLock()
{
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("acquireWakeLock");
    }
}

void Android::releaseWakeLock()
{
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("releaseWakeLock");
    }
}

void Android::acquireScreenLock()
{
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("acquireScreenLock");
    }
}

void Android::releaseScreenLock()
{
    QJniObject activity = QJniObject::callStaticObjectMethod("org/qtproject/qt/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("releaseScreenLock");
    }
}

void Android::selectDocumentTree(const std::function<void(const QString&)>& callback)
{
    static constexpr jint readPermission = 0x00000001;
    static constexpr jint writePermission = 0x00000002;
    static constexpr jint persistablePermission = 0x00000040;
    static constexpr jint prefixPermission = 0x00000080;

    const QJniObject action = QJniObject::fromString("android.intent.action.OPEN_DOCUMENT_TREE");
    QJniObject intent("android/content/Intent", "(Ljava/lang/String;)V", action.object<jstring>());
    intent.callObjectMethod(
        "addFlags", "(I)Landroid/content/Intent;",
        readPermission | writePermission | persistablePermission | prefixPermission);

    QtAndroidPrivate::startActivity(intent, nextDocumentTreeRequestCode(), [callback](int, int resultCode, const QJniObject& data) {
        QString selectedUri;

        if ((resultCode == -1) && data.isValid())
        {
            const QJniObject uri = data.callObjectMethod("getData", "()Landroid/net/Uri;");
            const QJniObject activity = androidActivity();

            if (uri.isValid() && activity.isValid())
            {
                const QJniObject resolver = activity.callObjectMethod(
                    "getContentResolver", "()Landroid/content/ContentResolver;");
                const jint grantedFlags = data.callMethod<jint>("getFlags") & 0x00000003;

                if (resolver.isValid()) {
                    resolver.callMethod<void>(
                        "takePersistableUriPermission", "(Landroid/net/Uri;I)V",
                        uri.object<jobject>(), grantedFlags);
                }

                selectedUri = uri.toString();
            }
        }

        if (callback) {
            callback(selectedUri);
        }
    });
}

bool Android::copyFileToDocumentTree(const QString& sourceFileName,
                                     const QString& treeUri,
                                     const QString& displayName,
                                     const QString& mimeType,
                                     QString *errorMessage)
{
    QFile source(sourceFileName);
    if (!source.open(QIODevice::ReadOnly))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot open %1: %2").arg(sourceFileName, source.errorString());
        }
        return false;
    }

    const QJniObject activity = androidActivity();
    const QJniObject tree = uriFromString(treeUri);
    if (!activity.isValid() || !tree.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("The Android output folder is no longer available.");
        }
        return false;
    }

    const QJniObject resolver = activity.callObjectMethod(
        "getContentResolver", "()Landroid/content/ContentResolver;");
    const QJniObject treeDocumentId = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "getTreeDocumentId",
        "(Landroid/net/Uri;)Ljava/lang/String;", tree.object<jobject>());
    if (!resolver.isValid() || !treeDocumentId.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot access the selected Android output folder.");
        }
        return false;
    }
    const QJniObject parent = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "buildDocumentUriUsingTree",
        "(Landroid/net/Uri;Ljava/lang/String;)Landroid/net/Uri;",
        tree.object<jobject>(), treeDocumentId.object<jstring>());
    if (!parent.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot access the selected Android output folder.");
        }
        return false;
    }
    const QJniObject mime = QJniObject::fromString(mimeType);
    const QJniObject name = QJniObject::fromString(displayName);
    const QJniObject document = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "createDocument",
        "(Landroid/content/ContentResolver;Landroid/net/Uri;Ljava/lang/String;Ljava/lang/String;)Landroid/net/Uri;",
        resolver.object<jobject>(), parent.object<jobject>(), mime.object<jstring>(), name.object<jstring>());

    if (!document.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot create %1 in the selected Android folder.").arg(displayName);
        }
        return false;
    }

    const QJniObject mode = QJniObject::fromString("w");
    QJniObject descriptor = resolver.callObjectMethod(
        "openFileDescriptor", "(Landroid/net/Uri;Ljava/lang/String;)Landroid/os/ParcelFileDescriptor;",
        document.object<jobject>(), mode.object<jstring>());
    const int fd = descriptor.isValid() ? descriptor.callMethod<jint>("getFd") : -1;
    QFile destination;
    bool ok = (fd >= 0) && destination.open(fd, QIODevice::WriteOnly, QFileDevice::DontCloseHandle);

    while (ok && !source.atEnd())
    {
        const QByteArray chunk = source.read(1024 * 1024);
        ok = !chunk.isEmpty() && (destination.write(chunk) == chunk.size());
    }

    if (ok) {
        ok = destination.flush();
    }
    destination.close();
    if (descriptor.isValid()) {
        descriptor.callMethod<void>("close");
    }

    if (!ok)
    {
        QJniObject::callStaticMethod<jboolean>(
            "android/provider/DocumentsContract", "deleteDocument",
            "(Landroid/content/ContentResolver;Landroid/net/Uri;)Z",
            resolver.object<jobject>(), document.object<jobject>());
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot copy %1 to the selected Android folder.").arg(displayName);
        }
    }

    return ok;
}

#else // QT_VERSION

#include <QtAndroid>
#include <QAndroidIntent>
#include <QAndroidJniObject>
#include <QAndroidJniEnvironment>
#include <QAndroidActivityResultReceiver>

namespace {

QAndroidJniObject androidActivity()
{
    return QAndroidJniObject::callStaticObjectMethod(
        "org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
}

QAndroidJniObject uriFromString(const QString& value)
{
    const QAndroidJniObject string = QAndroidJniObject::fromString(value);
    return QAndroidJniObject::callStaticObjectMethod(
        "android/net/Uri", "parse", "(Ljava/lang/String;)Landroid/net/Uri;", string.object<jstring>());
}

} // namespace

void Android::sendIntent()
{
    QAndroidJniObject url = QAndroidJniObject::fromString("iqsrc://-f 1090000000 -p 1234 -s 2000000 -a 127.0.0.1 -g 100");
    QAndroidJniObject intent = QAndroidJniObject::callStaticObjectMethod("android/content/Intent", "parseUri", "(Ljava/lang/String;I)Landroid/content/Intent;", url.object<jstring>(), 0x00000001);   // Creates Intent(ACTION_VIEW, url)
    QtAndroid::startActivity(intent, 0, [](int requestCode, int resultCode, const QAndroidJniObject &data) {
        (void) data;

        qDebug() << "MainCore::sendIntent " << requestCode << resultCode;
    });
}

QStringList Android::listUSBDeviceSerials(int vid, int pid)
{
    QStringList serials;
    QAndroidJniEnvironment jniEnv;

    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid())
    {
        QAndroidJniObject serialsObj = activity.callObjectMethod("listUSBDeviceSerials", "(II)[Ljava/lang/String;", vid, pid);
        int serialsLen = jniEnv->GetArrayLength(serialsObj.object<jarray>());
        for (int i = 0; i < serialsLen; i++)
        {
            QAndroidJniObject arrayElement = jniEnv->GetObjectArrayElement(serialsObj.object<jobjectArray>(), i);
            QString serial = arrayElement.toString();
            serials.append(serial);
        }
    }

    return serials;
}

int Android::openUSBDevice(const QString &serial)
{
    int fd = -1;
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid())
    {
        QAndroidJniObject serialsObj = QAndroidJniObject::fromString(serial);
        fd = activity.callMethod<jint>("openUSBDevice", "(Ljava/lang/String;)I", serialsObj.object<jstring>());
    }

    return fd;
}

void Android::closeUSBDevice(int fd)
{
    if (fd >= 0)
    {
        QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
        if (activity.isValid()) {
            activity.callMethod<void>("closeUSBDevice", "(I)V", fd);
        }
    }
}

void Android::moveTaskToBack()
{
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<jboolean>("moveTaskToBack", "(Z)Z", true);
    }
}

void Android::acquireWakeLock()
{
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("acquireWakeLock");
    }
}

void Android::releaseWakeLock()
{
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("releaseWakeLock");
    }
}

void Android::acquireScreenLock()
{
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("acquireScreenLock");
    }
}

void Android::releaseScreenLock()
{
    QAndroidJniObject activity = QAndroidJniObject::callStaticObjectMethod("org/qtproject/qt5/android/QtNative", "activity", "()Landroid/app/Activity;");
    if (activity.isValid()) {
        activity.callMethod<void>("releaseScreenLock");
    }
}

void Android::selectDocumentTree(const std::function<void(const QString&)>& callback)
{
    static constexpr jint permissionFlags = 0x00000001 | 0x00000002 | 0x00000040 | 0x00000080;
    const QAndroidJniObject action = QAndroidJniObject::fromString("android.intent.action.OPEN_DOCUMENT_TREE");
    QAndroidJniObject intent("android/content/Intent", "(Ljava/lang/String;)V", action.object<jstring>());
    intent.callObjectMethod("addFlags", "(I)Landroid/content/Intent;", permissionFlags);

    QtAndroid::startActivity(intent, nextDocumentTreeRequestCode(), [callback](int, int resultCode, const QAndroidJniObject& data) {
        QString selectedUri;
        if ((resultCode == -1) && data.isValid())
        {
            const QAndroidJniObject uri = data.callObjectMethod("getData", "()Landroid/net/Uri;");
            const QAndroidJniObject activity = androidActivity();
            if (uri.isValid() && activity.isValid())
            {
                const QAndroidJniObject resolver = activity.callObjectMethod(
                    "getContentResolver", "()Landroid/content/ContentResolver;");
                const jint grantedFlags = data.callMethod<jint>("getFlags") & 0x00000003;
                if (resolver.isValid()) {
                    resolver.callMethod<void>(
                        "takePersistableUriPermission", "(Landroid/net/Uri;I)V",
                        uri.object<jobject>(), grantedFlags);
                }
                selectedUri = uri.toString();
            }
        }
        if (callback) {
            callback(selectedUri);
        }
    });
}

bool Android::copyFileToDocumentTree(const QString& sourceFileName,
                                     const QString& treeUri,
                                     const QString& displayName,
                                     const QString& mimeType,
                                     QString *errorMessage)
{
    QFile source(sourceFileName);
    if (!source.open(QIODevice::ReadOnly))
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot open %1: %2").arg(sourceFileName, source.errorString());
        }
        return false;
    }

    const QAndroidJniObject activity = androidActivity();
    const QAndroidJniObject tree = uriFromString(treeUri);
    if (!activity.isValid() || !tree.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("The Android output folder is no longer available.");
        }
        return false;
    }

    const QAndroidJniObject resolver = activity.callObjectMethod(
        "getContentResolver", "()Landroid/content/ContentResolver;");
    const QAndroidJniObject treeDocumentId = QAndroidJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "getTreeDocumentId",
        "(Landroid/net/Uri;)Ljava/lang/String;", tree.object<jobject>());
    if (!resolver.isValid() || !treeDocumentId.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot access the selected Android output folder.");
        }
        return false;
    }
    const QAndroidJniObject parent = QAndroidJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "buildDocumentUriUsingTree",
        "(Landroid/net/Uri;Ljava/lang/String;)Landroid/net/Uri;",
        tree.object<jobject>(), treeDocumentId.object<jstring>());
    if (!parent.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot access the selected Android output folder.");
        }
        return false;
    }
    const QAndroidJniObject mime = QAndroidJniObject::fromString(mimeType);
    const QAndroidJniObject name = QAndroidJniObject::fromString(displayName);
    const QAndroidJniObject document = QAndroidJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "createDocument",
        "(Landroid/content/ContentResolver;Landroid/net/Uri;Ljava/lang/String;Ljava/lang/String;)Landroid/net/Uri;",
        resolver.object<jobject>(), parent.object<jobject>(), mime.object<jstring>(), name.object<jstring>());

    if (!document.isValid())
    {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot create %1 in the selected Android folder.").arg(displayName);
        }
        return false;
    }

    const QAndroidJniObject mode = QAndroidJniObject::fromString("w");
    QAndroidJniObject descriptor = resolver.callObjectMethod(
        "openFileDescriptor", "(Landroid/net/Uri;Ljava/lang/String;)Landroid/os/ParcelFileDescriptor;",
        document.object<jobject>(), mode.object<jstring>());
    const int fd = descriptor.isValid() ? descriptor.callMethod<jint>("getFd") : -1;
    QFile destination;
    bool ok = (fd >= 0) && destination.open(fd, QIODevice::WriteOnly, QFileDevice::DontCloseHandle);
    while (ok && !source.atEnd())
    {
        const QByteArray chunk = source.read(1024 * 1024);
        ok = !chunk.isEmpty() && (destination.write(chunk) == chunk.size());
    }
    if (ok) {
        ok = destination.flush();
    }
    destination.close();
    if (descriptor.isValid()) {
        descriptor.callMethod<void>("close");
    }

    if (!ok)
    {
        QAndroidJniObject::callStaticMethod<jboolean>(
            "android/provider/DocumentsContract", "deleteDocument",
            "(Landroid/content/ContentResolver;Landroid/net/Uri;)Z",
            resolver.object<jobject>(), document.object<jobject>());
        if (errorMessage) {
            *errorMessage = QStringLiteral("Cannot copy %1 to the selected Android folder.").arg(displayName);
        }
    }
    return ok;
}

#endif // QT6

// Redirect qDebug/qWarning to Android log, so we can view remotely with adb
void Android::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    QString report = msg;
    if (context.file && !QString(context.file).isEmpty())
    {
        report += " in file ";
        report += QString(context.file);
        report += " line ";
        report += QString::number(context.line);
    }
    if (context.function && !QString(context.function).isEmpty())
    {
        report += +" function ";
        report += QString(context.function);
    }
    const char * const local = report.toLocal8Bit().constData();
    const char * const applicationName = "sdrangel";
    int ret;
    switch (type)
    {
    case QtDebugMsg:
        ret = __android_log_write(ANDROID_LOG_DEBUG, applicationName, local);
        break;
    case QtInfoMsg:
        ret = __android_log_write(ANDROID_LOG_INFO, applicationName, local);
        break;
    case QtWarningMsg:
        ret = __android_log_write(ANDROID_LOG_WARN, applicationName, local);
        break;
    case QtCriticalMsg:
        ret = __android_log_write(ANDROID_LOG_ERROR, applicationName, local);
        break;
    case QtFatalMsg:
    default:
        ret = __android_log_write(ANDROID_LOG_FATAL, applicationName, local);
        abort();
    }
    if (ret < 0) {
        __android_log_write(ANDROID_LOG_ERROR, applicationName, "Error writing to log");
    }
}

#endif // ANDROID
