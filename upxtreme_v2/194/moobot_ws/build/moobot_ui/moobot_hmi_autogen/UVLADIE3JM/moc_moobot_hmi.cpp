/****************************************************************************
** Meta object code from reading C++ file 'moobot_hmi.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/moobot_ui/src/moobot_hmi.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'moobot_hmi.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Moobot_hmi_t {
    QByteArrayData data[8];
    char stringdata0[85];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Moobot_hmi_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Moobot_hmi_t qt_meta_stringdata_Moobot_hmi = {
    {
QT_MOC_LITERAL(0, 0, 10), // "Moobot_hmi"
QT_MOC_LITERAL(1, 11, 10), // "stopSignal"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 9), // "isStopped"
QT_MOC_LITERAL(4, 33, 26), // "on_startStopButton_pressed"
QT_MOC_LITERAL(5, 60, 10), // "changeMode"
QT_MOC_LITERAL(6, 71, 4), // "mode"
QT_MOC_LITERAL(7, 76, 8) // "callSpin"

    },
    "Moobot_hmi\0stopSignal\0\0isStopped\0"
    "on_startStopButton_pressed\0changeMode\0"
    "mode\0callSpin"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Moobot_hmi[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   37,    2, 0x0a /* Public */,
       5,    1,   38,    2, 0x0a /* Public */,
       7,    0,   41,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,

       0        // eod
};

void Moobot_hmi::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Moobot_hmi *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->stopSignal((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_startStopButton_pressed(); break;
        case 2: _t->changeMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->callSpin(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Moobot_hmi::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Moobot_hmi::stopSignal)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Moobot_hmi::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_Moobot_hmi.data,
    qt_meta_data_Moobot_hmi,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Moobot_hmi::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Moobot_hmi::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Moobot_hmi.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int Moobot_hmi::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void Moobot_hmi::stopSignal(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
