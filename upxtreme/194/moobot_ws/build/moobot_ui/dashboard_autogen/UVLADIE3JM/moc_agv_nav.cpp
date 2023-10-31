/****************************************************************************
** Meta object code from reading C++ file 'agv_nav.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/moobot_ui/src/agv_nav.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'agv_nav.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Agv_nav_t {
    QByteArrayData data[17];
    char stringdata0[160];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Agv_nav_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Agv_nav_t qt_meta_stringdata_Agv_nav = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Agv_nav"
QT_MOC_LITERAL(1, 8, 22), // "goWithPointErrorSignal"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 5), // "error"
QT_MOC_LITERAL(4, 38, 11), // "station_num"
QT_MOC_LITERAL(5, 50, 17), // "goWithPointIsDone"
QT_MOC_LITERAL(6, 68, 4), // "done"
QT_MOC_LITERAL(7, 73, 13), // "removeStation"
QT_MOC_LITERAL(8, 87, 3), // "arg"
QT_MOC_LITERAL(9, 91, 15), // "relocateStation"
QT_MOC_LITERAL(10, 107, 15), // "goWithPointSlot"
QT_MOC_LITERAL(11, 123, 11), // "std::string"
QT_MOC_LITERAL(12, 135, 1), // "x"
QT_MOC_LITERAL(13, 137, 1), // "y"
QT_MOC_LITERAL(14, 139, 1), // "z"
QT_MOC_LITERAL(15, 141, 8), // "stopSlot"
QT_MOC_LITERAL(16, 150, 9) // "isStopped"

    },
    "Agv_nav\0goWithPointErrorSignal\0\0error\0"
    "station_num\0goWithPointIsDone\0done\0"
    "removeStation\0arg\0relocateStation\0"
    "goWithPointSlot\0std::string\0x\0y\0z\0"
    "stopSlot\0isStopped"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Agv_nav[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   44,    2, 0x06 /* Public */,
       5,    2,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   54,    2, 0x0a /* Public */,
       9,    1,   57,    2, 0x0a /* Public */,
      10,    5,   60,    2, 0x0a /* Public */,
      15,    1,   71,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    3,    4,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,    6,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, 0x80000000 | 11, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Int,    8,   12,   13,   14,    4,
    QMetaType::Void, QMetaType::Bool,   16,

       0        // eod
};

void Agv_nav::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Agv_nav *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->goWithPointErrorSignal((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->goWithPointIsDone((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->removeStation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->relocateStation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->goWithPointSlot((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 5: _t->stopSlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Agv_nav::*)(QString , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agv_nav::goWithPointErrorSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Agv_nav::*)(bool , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Agv_nav::goWithPointIsDone)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Agv_nav::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_Agv_nav.data,
    qt_meta_data_Agv_nav,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Agv_nav::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Agv_nav::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Agv_nav.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Agv_nav::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void Agv_nav::goWithPointErrorSignal(QString _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Agv_nav::goWithPointIsDone(bool _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
