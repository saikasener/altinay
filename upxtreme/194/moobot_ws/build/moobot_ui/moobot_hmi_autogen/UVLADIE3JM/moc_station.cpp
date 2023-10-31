/****************************************************************************
** Meta object code from reading C++ file 'station.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/moobot_ui/src/station.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'station.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Station_t {
    QByteArrayData data[23];
    char stringdata0[319];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Station_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Station_t qt_meta_stringdata_Station = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Station"
QT_MOC_LITERAL(1, 8, 12), // "removeSignal"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 3), // "num"
QT_MOC_LITERAL(4, 26, 14), // "relocateSignal"
QT_MOC_LITERAL(5, 41, 11), // "pointSignal"
QT_MOC_LITERAL(6, 53, 11), // "std::string"
QT_MOC_LITERAL(7, 65, 8), // "go_point"
QT_MOC_LITERAL(8, 74, 1), // "x"
QT_MOC_LITERAL(9, 76, 1), // "y"
QT_MOC_LITERAL(10, 78, 1), // "z"
QT_MOC_LITERAL(11, 80, 11), // "station_num"
QT_MOC_LITERAL(12, 92, 12), // "enableRemove"
QT_MOC_LITERAL(13, 105, 14), // "isRelocateDone"
QT_MOC_LITERAL(14, 120, 20), // "goWithPointErrorSlot"
QT_MOC_LITERAL(15, 141, 5), // "error"
QT_MOC_LITERAL(16, 147, 21), // "goWithPointIsDoneSlot"
QT_MOC_LITERAL(17, 169, 4), // "done"
QT_MOC_LITERAL(18, 174, 25), // "on_relocateButton_clicked"
QT_MOC_LITERAL(19, 200, 23), // "on_removeButton_clicked"
QT_MOC_LITERAL(20, 224, 26), // "on_goStationButton_clicked"
QT_MOC_LITERAL(21, 251, 35), // "on_goStationWithPointButton_p..."
QT_MOC_LITERAL(22, 287, 31) // "on_orientation_text_textChanged"

    },
    "Station\0removeSignal\0\0num\0relocateSignal\0"
    "pointSignal\0std::string\0go_point\0x\0y\0"
    "z\0station_num\0enableRemove\0isRelocateDone\0"
    "goWithPointErrorSlot\0error\0"
    "goWithPointIsDoneSlot\0done\0"
    "on_relocateButton_clicked\0"
    "on_removeButton_clicked\0"
    "on_goStationButton_clicked\0"
    "on_goStationWithPointButton_pressed\0"
    "on_orientation_text_textChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Station[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       4,    1,   72,    2, 0x06 /* Public */,
       5,    5,   75,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      12,    1,   86,    2, 0x0a /* Public */,
      14,    2,   89,    2, 0x0a /* Public */,
      16,    2,   94,    2, 0x0a /* Public */,
      18,    0,   99,    2, 0x08 /* Private */,
      19,    0,  100,    2, 0x08 /* Private */,
      20,    0,  101,    2, 0x08 /* Private */,
      21,    0,  102,    2, 0x08 /* Private */,
      22,    0,  103,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, 0x80000000 | 6, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Int,    7,    8,    9,   10,   11,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   13,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,   15,   11,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,   17,   11,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Station::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Station *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->removeSignal((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 1: _t->relocateSignal((*reinterpret_cast< const int(*)>(_a[1]))); break;
        case 2: _t->pointSignal((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const float(*)>(_a[2])),(*reinterpret_cast< const float(*)>(_a[3])),(*reinterpret_cast< const float(*)>(_a[4])),(*reinterpret_cast< const int(*)>(_a[5]))); break;
        case 3: _t->enableRemove((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->goWithPointErrorSlot((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 5: _t->goWithPointIsDoneSlot((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 6: _t->on_relocateButton_clicked(); break;
        case 7: _t->on_removeButton_clicked(); break;
        case 8: _t->on_goStationButton_clicked(); break;
        case 9: _t->on_goStationWithPointButton_pressed(); break;
        case 10: _t->on_orientation_text_textChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Station::*)(const int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Station::removeSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Station::*)(const int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Station::relocateSignal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Station::*)(const std::string , const float , const float , const float , const int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Station::pointSignal)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Station::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_Station.data,
    qt_meta_data_Station,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Station::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Station::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Station.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Station::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void Station::removeSignal(const int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Station::relocateSignal(const int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Station::pointSignal(const std::string _t1, const float _t2, const float _t3, const float _t4, const int _t5)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
