#include "../../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[21];
    char stringdata0[413];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 20), // "slot_btnBackToOrigin"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 17), // "slot_btnLoadModel"
QT_MOC_LITERAL(4, 51, 16), // "slot_btnGetTrunk"
QT_MOC_LITERAL(5, 68, 20), // "slot_btnConnectGraph"
QT_MOC_LITERAL(6, 89, 16), // "slot_btnMinGraph"
QT_MOC_LITERAL(7, 106, 16), // "slot_btnGetBins1"
QT_MOC_LITERAL(8, 123, 16), // "slot_btnGetBins2"
QT_MOC_LITERAL(9, 140, 23), // "slot_btnGetTreeSkeleton"
QT_MOC_LITERAL(10, 164, 24), // "slot_btnOptimizeSkeleton"
QT_MOC_LITERAL(11, 189, 26), // "slot_btnSaveTreeStructures"
QT_MOC_LITERAL(12, 216, 33), // "slot_ckb_DisplaySkeletonWithC..."
QT_MOC_LITERAL(13, 250, 23), // "slot_btnSaveDepthBuffer"
QT_MOC_LITERAL(14, 274, 22), // "slot_cbxSetDisplayMode"
QT_MOC_LITERAL(15, 297, 32), // "slot_spinChangeDisplayParameters"
QT_MOC_LITERAL(16, 330, 33), // "slot_compareSkeletonWithOrigi..."
QT_MOC_LITERAL(17, 364, 17), // "slot_startRoaming"
QT_MOC_LITERAL(18, 382, 26), // "slot_updateCurCaputerCount"
QT_MOC_LITERAL(19, 409, 1), // "a"
QT_MOC_LITERAL(20, 411, 1) // "b"

    },
    "MainWindow\0slot_btnBackToOrigin\0\0"
    "slot_btnLoadModel\0slot_btnGetTrunk\0"
    "slot_btnConnectGraph\0slot_btnMinGraph\0"
    "slot_btnGetBins1\0slot_btnGetBins2\0"
    "slot_btnGetTreeSkeleton\0"
    "slot_btnOptimizeSkeleton\0"
    "slot_btnSaveTreeStructures\0"
    "slot_ckb_DisplaySkeletonWithColor\0"
    "slot_btnSaveDepthBuffer\0slot_cbxSetDisplayMode\0"
    "slot_spinChangeDisplayParameters\0"
    "slot_compareSkeletonWithOriginPts\0"
    "slot_startRoaming\0slot_updateCurCaputerCount\0"
    "a\0b"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   99,    2, 0x09 /* Protected */,
       3,    0,  100,    2, 0x09 /* Protected */,
       4,    0,  101,    2, 0x09 /* Protected */,
       5,    0,  102,    2, 0x09 /* Protected */,
       6,    0,  103,    2, 0x09 /* Protected */,
       7,    0,  104,    2, 0x09 /* Protected */,
       8,    0,  105,    2, 0x09 /* Protected */,
       9,    0,  106,    2, 0x09 /* Protected */,
      10,    0,  107,    2, 0x09 /* Protected */,
      11,    0,  108,    2, 0x09 /* Protected */,
      12,    0,  109,    2, 0x09 /* Protected */,
      13,    0,  110,    2, 0x09 /* Protected */,
      14,    0,  111,    2, 0x09 /* Protected */,
      15,    0,  112,    2, 0x09 /* Protected */,
      16,    0,  113,    2, 0x09 /* Protected */,
      17,    0,  114,    2, 0x09 /* Protected */,
      18,    2,  115,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   19,   20,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->slot_btnBackToOrigin(); break;
        case 1: _t->slot_btnLoadModel(); break;
        case 2: _t->slot_btnGetTrunk(); break;
        case 3: _t->slot_btnConnectGraph(); break;
        case 4: _t->slot_btnMinGraph(); break;
        case 5: _t->slot_btnGetBins1(); break;
        case 6: _t->slot_btnGetBins2(); break;
        case 7: _t->slot_btnGetTreeSkeleton(); break;
        case 8: _t->slot_btnOptimizeSkeleton(); break;
        case 9: _t->slot_btnSaveTreeStructures(); break;
        case 10: _t->slot_ckb_DisplaySkeletonWithColor(); break;
        case 11: _t->slot_btnSaveDepthBuffer(); break;
        case 12: _t->slot_cbxSetDisplayMode(); break;
        case 13: _t->slot_spinChangeDisplayParameters(); break;
        case 14: _t->slot_compareSkeletonWithOriginPts(); break;
        case 15: _t->slot_startRoaming(); break;
        case 16: _t->slot_updateCurCaputerCount((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
