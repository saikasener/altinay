/********************************************************************************
** Form generated from reading UI file 'statuspanel.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_STATUSPANEL_H
#define UI_STATUSPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_StatusPanel
{
public:
    QLabel *imu_status;
    QLabel *label_2;
    QLabel *led_status;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *scanner_status;
    QLabel *map_status;
    QLabel *label_6;

    void setupUi(QWidget *StatusPanel)
    {
        if (StatusPanel->objectName().isEmpty())
            StatusPanel->setObjectName(QString::fromUtf8("StatusPanel"));
        StatusPanel->resize(539, 59);
        imu_status = new QLabel(StatusPanel);
        imu_status->setObjectName(QString::fromUtf8("imu_status"));
        imu_status->setGeometry(QRect(10, 10, 35, 35));
        imu_status->setStyleSheet(QString::fromUtf8(""));
        label_2 = new QLabel(StatusPanel);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(50, 10, 71, 35));
        label_2->setStyleSheet(QString::fromUtf8("\n"
"font: 75 11pt \"Ubuntu\";\n"
""));
        led_status = new QLabel(StatusPanel);
        led_status->setObjectName(QString::fromUtf8("led_status"));
        led_status->setGeometry(QRect(130, 10, 35, 35));
        led_status->setStyleSheet(QString::fromUtf8("font: 57 italic 11pt \"Ubuntu\";"));
        label_4 = new QLabel(StatusPanel);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(170, 10, 71, 35));
        label_4->setStyleSheet(QString::fromUtf8("\n"
"font: 75 11pt \"Ubuntu\";"));
        label_5 = new QLabel(StatusPanel);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(290, 10, 81, 35));
        label_5->setStyleSheet(QString::fromUtf8("\n"
"font: 75 11pt \"Ubuntu\";"));
        scanner_status = new QLabel(StatusPanel);
        scanner_status->setObjectName(QString::fromUtf8("scanner_status"));
        scanner_status->setGeometry(QRect(250, 10, 35, 35));
        scanner_status->setStyleSheet(QString::fromUtf8("font: 57 italic 11pt \"Ubuntu\";"));
        map_status = new QLabel(StatusPanel);
        map_status->setObjectName(QString::fromUtf8("map_status"));
        map_status->setGeometry(QRect(390, 10, 35, 35));
        map_status->setStyleSheet(QString::fromUtf8("font: 57 italic 11pt \"Ubuntu\";"));
        label_6 = new QLabel(StatusPanel);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(430, 10, 81, 35));
        label_6->setStyleSheet(QString::fromUtf8("\n"
"font: 75 11pt \"Ubuntu\";"));

        retranslateUi(StatusPanel);

        QMetaObject::connectSlotsByName(StatusPanel);
    } // setupUi

    void retranslateUi(QWidget *StatusPanel)
    {
        StatusPanel->setWindowTitle(QApplication::translate("StatusPanel", "Form", nullptr));
        imu_status->setText(QString());
        label_2->setText(QApplication::translate("StatusPanel", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">IMU</span></p></body></html>", nullptr));
        led_status->setText(QString());
        label_4->setText(QApplication::translate("StatusPanel", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">Led</span></p></body></html>", nullptr));
        label_5->setText(QApplication::translate("StatusPanel", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">Scanner</span></p></body></html>", nullptr));
        scanner_status->setText(QString());
        map_status->setText(QApplication::translate("StatusPanel", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
        label_6->setText(QApplication::translate("StatusPanel", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:600;\">Map</span></p></body></html>", nullptr));
    } // retranslateUi

};

namespace Ui {
    class StatusPanel: public Ui_StatusPanel {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STATUSPANEL_H
