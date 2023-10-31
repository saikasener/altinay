/****************************************************************************
** Meta object code from reading C++ file 'dashboard.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/moobot_ui/src/dashboard.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dashboard.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Dashboard_t {
    QByteArrayData data[15];
    char stringdata0[298];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Dashboard_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Dashboard_t qt_meta_stringdata_Dashboard = {
    {
QT_MOC_LITERAL(0, 0, 9), // "Dashboard"
QT_MOC_LITERAL(1, 10, 10), // "stopSignal"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 9), // "isStopped"
QT_MOC_LITERAL(4, 32, 26), // "on_startStopButton_pressed"
QT_MOC_LITERAL(5, 59, 26), // "on_save_map_button_pressed"
QT_MOC_LITERAL(6, 86, 32), // "on_run_navigation_button_pressed"
QT_MOC_LITERAL(7, 119, 33), // "on_stop_navigation_button_pre..."
QT_MOC_LITERAL(8, 153, 31), // "on_start_mapping_button_pressed"
QT_MOC_LITERAL(9, 185, 30), // "on_stop_mapping_button_pressed"
QT_MOC_LITERAL(10, 216, 28), // "on_start_loop_button_pressed"
QT_MOC_LITERAL(11, 245, 27), // "on_stop_loop_button_pressed"
QT_MOC_LITERAL(12, 273, 10), // "changeMode"
QT_MOC_LITERAL(13, 284, 4), // "mode"
QT_MOC_LITERAL(14, 289, 8) // "callSpin"

    },
    "Dashboard\0stopSignal\0\0isStopped\0"
    "on_startStopButton_pressed\0"
    "on_save_map_button_pressed\0"
    "on_run_navigation_button_pressed\0"
    "on_stop_navigation_button_pressed\0"
    "on_start_mapping_button_pressed\0"
    "on_stop_mapping_button_pressed\0"
    "on_start_loop_button_pressed\0"
    "on_stop_loop_button_pressed\0changeMode\0"
    "mode\0callSpin"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Dashboard[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   72,    2, 0x0a /* Public */,
       5,    0,   73,    2, 0x0a /* Public */,
       6,    0,   74,    2, 0x0a /* Public */,
       7,    0,   75,    2, 0x0a /* Public */,
       8,    0,   76,    2, 0x0a /* Public */,
       9,    0,   77,    2, 0x0a /* Public */,
      10,    0,   78,    2, 0x0a /* Public */,
      11,    0,   79,    2, 0x0a /* Public */,
      12,    1,   80,    2, 0x0a /* Public */,
      14,    0,   83,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   13,
    QMetaType::Void,

       0        // eod
};

void Dashboard::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Dashboard *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->stopSignal((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_startStopButton_pressed(); break;
        case 2: _t->on_save_map_button_pressed(); break;
        case 3: _t->on_run_navigation_button_pressed(); break;
        case 4: _t->on_stop_navigation_button_pressed(); break;
        case 5: _t->on_start_mapping_button_pressed(); break;
        case 6: _t->on_stop_mapping_button_pressed(); break;
        case 7: _t->on_start_loop_button_pressed(); break;
        case 8: _t->on_stop_loop_button_pressed(); break;
        case 9: _t->changeMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->callSpin(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Dashboard::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Dashboard::stopSignal)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Dashboard::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_Dashboard.data,
    qt_meta_data_Dashboard,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Dashboard::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Dashboard::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Dashboard.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int Dashboard::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
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
void Dashboard::stopSignal(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
