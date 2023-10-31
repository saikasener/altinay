/********************************************************************************
** Form generated from reading UI file 'joypad.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOYPAD_H
#define UI_JOYPAD_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Joypad
{
public:
    QPushButton *set_speed_button;
    QDoubleSpinBox *angular_speed_entered;
    QLabel *JoyPad;
    QDoubleSpinBox *linear_speed_entered;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_5;
    QLabel *label_4;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *linear_val;
    QLabel *angular_val;
    QDoubleSpinBox *min_linear_speed_entered;
    QDoubleSpinBox *max_angular_speed_entered;
    QPushButton *set_speed_button_min;
    QWidget *layoutWidget_16;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_26;
    QLabel *label_27;
    QDoubleSpinBox *min_angular_speed_entered;
    QPushButton *set_speed_button_max;
    QDoubleSpinBox *max_linear_speed_entered;
    QWidget *layoutWidget_17;
    QHBoxLayout *horizontalLayout_19;
    QLabel *label_9;
    QLabel *label_10;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_25;
    QLabel *label_28;
    QLabel *label_11;
    QSpinBox *reduce_ratio;

    void setupUi(QWidget *Joypad)
    {
        if (Joypad->objectName().isEmpty())
            Joypad->setObjectName(QString::fromUtf8("Joypad"));
        Joypad->resize(360, 602);
        set_speed_button = new QPushButton(Joypad);
        set_speed_button->setObjectName(QString::fromUtf8("set_speed_button"));
        set_speed_button->setGeometry(QRect(300, 410, 50, 29));
        angular_speed_entered = new QDoubleSpinBox(Joypad);
        angular_speed_entered->setObjectName(QString::fromUtf8("angular_speed_entered"));
        angular_speed_entered->setGeometry(QRect(180, 410, 69, 27));
        angular_speed_entered->setMinimum(-7.000000000000000);
        angular_speed_entered->setMaximum(7.000000000000000);
        angular_speed_entered->setSingleStep(0.010000000000000);
        JoyPad = new QLabel(Joypad);
        JoyPad->setObjectName(QString::fromUtf8("JoyPad"));
        JoyPad->setGeometry(QRect(60, 80, 230, 230));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(JoyPad->sizePolicy().hasHeightForWidth());
        JoyPad->setSizePolicy(sizePolicy);
        JoyPad->setStyleSheet(QString::fromUtf8(""));
        linear_speed_entered = new QDoubleSpinBox(Joypad);
        linear_speed_entered->setObjectName(QString::fromUtf8("linear_speed_entered"));
        linear_speed_entered->setGeometry(QRect(50, 410, 69, 27));
        linear_speed_entered->setMinimum(-20.000000000000000);
        linear_speed_entered->setMaximum(20.000000000000000);
        linear_speed_entered->setSingleStep(0.010000000000000);
        layoutWidget = new QWidget(Joypad);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 0, 311, 31));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout->addWidget(label_5);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout->addWidget(label_4);

        layoutWidget1 = new QWidget(Joypad);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(20, 30, 311, 41));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        linear_val = new QLabel(layoutWidget1);
        linear_val->setObjectName(QString::fromUtf8("linear_val"));

        horizontalLayout_2->addWidget(linear_val);

        angular_val = new QLabel(layoutWidget1);
        angular_val->setObjectName(QString::fromUtf8("angular_val"));

        horizontalLayout_2->addWidget(angular_val);

        min_linear_speed_entered = new QDoubleSpinBox(Joypad);
        min_linear_speed_entered->setObjectName(QString::fromUtf8("min_linear_speed_entered"));
        min_linear_speed_entered->setGeometry(QRect(50, 570, 69, 27));
        min_linear_speed_entered->setMinimum(-20.000000000000000);
        min_linear_speed_entered->setMaximum(20.000000000000000);
        min_linear_speed_entered->setSingleStep(0.100000000000000);
        max_angular_speed_entered = new QDoubleSpinBox(Joypad);
        max_angular_speed_entered->setObjectName(QString::fromUtf8("max_angular_speed_entered"));
        max_angular_speed_entered->setGeometry(QRect(180, 490, 69, 27));
        max_angular_speed_entered->setMinimum(0.000000000000000);
        max_angular_speed_entered->setMaximum(7.000000000000000);
        max_angular_speed_entered->setSingleStep(0.010000000000000);
        set_speed_button_min = new QPushButton(Joypad);
        set_speed_button_min->setObjectName(QString::fromUtf8("set_speed_button_min"));
        set_speed_button_min->setGeometry(QRect(300, 570, 50, 29));
        layoutWidget_16 = new QWidget(Joypad);
        layoutWidget_16->setObjectName(QString::fromUtf8("layoutWidget_16"));
        layoutWidget_16->setGeometry(QRect(0, 450, 301, 31));
        horizontalLayout_17 = new QHBoxLayout(layoutWidget_16);
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        horizontalLayout_17->setContentsMargins(0, 0, 0, 0);
        label_26 = new QLabel(layoutWidget_16);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        horizontalLayout_17->addWidget(label_26);

        label_27 = new QLabel(layoutWidget_16);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        horizontalLayout_17->addWidget(label_27);

        min_angular_speed_entered = new QDoubleSpinBox(Joypad);
        min_angular_speed_entered->setObjectName(QString::fromUtf8("min_angular_speed_entered"));
        min_angular_speed_entered->setGeometry(QRect(180, 570, 69, 27));
        min_angular_speed_entered->setMinimum(-7.000000000000000);
        min_angular_speed_entered->setMaximum(7.000000000000000);
        min_angular_speed_entered->setSingleStep(0.010000000000000);
        set_speed_button_max = new QPushButton(Joypad);
        set_speed_button_max->setObjectName(QString::fromUtf8("set_speed_button_max"));
        set_speed_button_max->setGeometry(QRect(300, 490, 50, 29));
        max_linear_speed_entered = new QDoubleSpinBox(Joypad);
        max_linear_speed_entered->setObjectName(QString::fromUtf8("max_linear_speed_entered"));
        max_linear_speed_entered->setGeometry(QRect(50, 490, 69, 27));
        max_linear_speed_entered->setMinimum(0.000000000000000);
        max_linear_speed_entered->setMaximum(20.000000000000000);
        max_linear_speed_entered->setSingleStep(0.100000000000000);
        layoutWidget_17 = new QWidget(Joypad);
        layoutWidget_17->setObjectName(QString::fromUtf8("layoutWidget_17"));
        layoutWidget_17->setGeometry(QRect(0, 370, 301, 31));
        horizontalLayout_19 = new QHBoxLayout(layoutWidget_17);
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        horizontalLayout_19->setContentsMargins(0, 0, 0, 0);
        label_9 = new QLabel(layoutWidget_17);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_19->addWidget(label_9);

        label_10 = new QLabel(layoutWidget_17);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_19->addWidget(label_10);

        layoutWidget2 = new QWidget(Joypad);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(0, 530, 301, 31));
        horizontalLayout_18 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        horizontalLayout_18->setContentsMargins(0, 0, 0, 0);
        label_25 = new QLabel(layoutWidget2);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        horizontalLayout_18->addWidget(label_25);

        label_28 = new QLabel(layoutWidget2);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        horizontalLayout_18->addWidget(label_28);

        label_11 = new QLabel(Joypad);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(170, 330, 111, 29));
        reduce_ratio = new QSpinBox(Joypad);
        reduce_ratio->setObjectName(QString::fromUtf8("reduce_ratio"));
        reduce_ratio->setGeometry(QRect(290, 330, 60, 27));
        reduce_ratio->setMaximum(100);
        reduce_ratio->setValue(100);

        retranslateUi(Joypad);

        QMetaObject::connectSlotsByName(Joypad);
    } // setupUi

    void retranslateUi(QWidget *Joypad)
    {
        Joypad->setWindowTitle(QApplication::translate("Joypad", "Joypad", nullptr));
        set_speed_button->setText(QApplication::translate("Joypad", "SET", nullptr));
        JoyPad->setText(QString());
        label_5->setText(QApplication::translate("Joypad", "<html> <head><body><p><span style=\" font-size:12pt; font-weight:600;\"><center>Linear Speed</center></span></p></body></head></html>", nullptr));
        label_4->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Angular Speed</span></p></body></html>", nullptr));
        linear_val->setText(QApplication::translate("Joypad", "<html> <head><body><p><span style=\" font-size:18pt; font-weight:600;\"><center>0</center></span></p></body></head></html>", nullptr));
        angular_val->setText(QApplication::translate("Joypad", "<html> <head><body><p><span style=\" font-size:18pt; font-weight:600;\"><center>0</center></span></p></body></head></html>", nullptr));
        set_speed_button_min->setText(QApplication::translate("Joypad", "SET", nullptr));
        label_26->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Max Linear Speed</span></p></body></html>", nullptr));
        label_27->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Max Angular Speed</span></p></body></html>", nullptr));
        set_speed_button_max->setText(QApplication::translate("Joypad", "SET", nullptr));
        label_9->setText(QApplication::translate("Joypad", "<html> <head><body><p><span style=\" font-size:12pt; font-weight:600;\"><center>Linear Speed</center></span></p></body></head></html>", nullptr));
        label_10->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Angular Speed</span></p></body></html>", nullptr));
        label_25->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Min Linear Speed</span></p></body></html>", nullptr));
        label_28->setText(QApplication::translate("Joypad", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt; font-weight:600;\">Min Angular Speed</span></p></body></html>", nullptr));
        label_11->setText(QApplication::translate("Joypad", "<html><head/><body><p><span style=\" font-size:12pt;\">Reduce Ratio:</span></p></body></html>", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Joypad: public Ui_Joypad {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOYPAD_H
