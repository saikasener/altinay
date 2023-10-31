#-------------------------------------------------
#
# Project created by QtCreator 2020-08-28T13:30:47
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = deneme_ui
TEMPLATE = app


SOURCES += main.cpp\
        dashboard.cpp \
    switch.cpp \
    statuspanel.cpp

HEADERS  += dashboard.h \
    switch.h \
    statuspanel.h

FORMS    += dashboard.ui \
    statuspanel.ui
