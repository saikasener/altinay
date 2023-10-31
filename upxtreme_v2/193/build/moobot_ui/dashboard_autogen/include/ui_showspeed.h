/********************************************************************************
** Form generated from reading UI file 'showspeed.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SHOWSPEED_H
#define UI_SHOWSPEED_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ShowSpeed
{
public:
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *angularSpeedMin;
    QLabel *label;
    QLabel *linearSpeedMin;
    QLabel *label_8;
    QLabel *label_2;
    QLabel *label_7;
    QLabel *linearSpeedAct;
    QLabel *angularSpeedDes;
    QLabel *angularSpeedAct;
    QLabel *angularSpeedMax;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *linearSpeedMax;
    QLabel *linearSpeedDes;

    void setupUi(QWidget *ShowSpeed)
    {
        if (ShowSpeed->objectName().isEmpty())
            ShowSpeed->setObjectName(QString::fromUtf8("ShowSpeed"));
        ShowSpeed->resize(321, 172);
        ShowSpeed->setStyleSheet(QString::fromUtf8(""));
        layoutWidget = new QWidget(ShowSpeed);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 0, 321, 171));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        angularSpeedMin = new QLabel(layoutWidget);
        angularSpeedMin->setObjectName(QString::fromUtf8("angularSpeedMin"));
        angularSpeedMin->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(angularSpeedMin, 4, 2, 1, 1);

        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setStyleSheet(QString::fromUtf8("font: 75 italic 11pt \"Ubuntu\";\n"
"text-decoration: underline;"));

        gridLayout->addWidget(label, 0, 1, 1, 1);

        linearSpeedMin = new QLabel(layoutWidget);
        linearSpeedMin->setObjectName(QString::fromUtf8("linearSpeedMin"));
        linearSpeedMin->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(linearSpeedMin, 4, 1, 1, 1);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout->addWidget(label_8, 4, 0, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setStyleSheet(QString::fromUtf8("font: 75 italic 11pt \"Ubuntu\";\n"
"text-decoration: underline;"));

        gridLayout->addWidget(label_2, 0, 2, 1, 1);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout->addWidget(label_7, 3, 0, 1, 1);

        linearSpeedAct = new QLabel(layoutWidget);
        linearSpeedAct->setObjectName(QString::fromUtf8("linearSpeedAct"));
        linearSpeedAct->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(linearSpeedAct, 1, 1, 1, 1);

        angularSpeedDes = new QLabel(layoutWidget);
        angularSpeedDes->setObjectName(QString::fromUtf8("angularSpeedDes"));
        angularSpeedDes->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(angularSpeedDes, 2, 2, 1, 1);

        angularSpeedAct = new QLabel(layoutWidget);
        angularSpeedAct->setObjectName(QString::fromUtf8("angularSpeedAct"));
        angularSpeedAct->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(angularSpeedAct, 1, 2, 1, 1);

        angularSpeedMax = new QLabel(layoutWidget);
        angularSpeedMax->setObjectName(QString::fromUtf8("angularSpeedMax"));
        angularSpeedMax->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(angularSpeedMax, 3, 2, 1, 1);

        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setStyleSheet(QString::fromUtf8("font: 75 italic 11pt \"Ubuntu\";\n"
"text-decoration: underline;"));

        gridLayout->addWidget(label_5, 1, 0, 1, 1);

        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 2, 0, 1, 1);

        linearSpeedMax = new QLabel(layoutWidget);
        linearSpeedMax->setObjectName(QString::fromUtf8("linearSpeedMax"));
        linearSpeedMax->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(linearSpeedMax, 3, 1, 1, 1);

        linearSpeedDes = new QLabel(layoutWidget);
        linearSpeedDes->setObjectName(QString::fromUtf8("linearSpeedDes"));
        linearSpeedDes->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        gridLayout->addWidget(linearSpeedDes, 2, 1, 1, 1);


        retranslateUi(ShowSpeed);

        QMetaObject::connectSlotsByName(ShowSpeed);
    } // setupUi

    void retranslateUi(QWidget *ShowSpeed)
    {
        ShowSpeed->setWindowTitle(QApplication::translate("ShowSpeed", "ShowSpeed", nullptr));
        angularSpeedMin->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
        label->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Linear (m/s)</span></p></body></html>", nullptr));
        linearSpeedMin->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
        label_8->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">MIN</span></p></body></html>", nullptr));
        label_2->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Angular (rad/s)</span></p></body></html>", nullptr));
        label_7->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">MAX</span></p></body></html>", nullptr));
        linearSpeedAct->setText(QString());
        angularSpeedDes->setText(QString());
        angularSpeedAct->setText(QString());
        angularSpeedMax->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
        label_5->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">ACTUAL</span></p></body></html>", nullptr));
        label_6->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">DESIRED</span></p></body></html>", nullptr));
        linearSpeedMax->setText(QApplication::translate("ShowSpeed", "<html><head/><body><p align=\"center\"><br/></p></body></html>", nullptr));
        linearSpeedDes->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ShowSpeed: public Ui_ShowSpeed {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SHOWSPEED_H
