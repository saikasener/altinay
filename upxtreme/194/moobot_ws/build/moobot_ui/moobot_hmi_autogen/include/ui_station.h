/********************************************************************************
** Form generated from reading UI file 'station.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_STATION_H
#define UI_STATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Station
{
public:
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QVBoxLayout *verticalLayout_2;
    QLabel *station_label;
    QLabel *x_label;
    QLabel *y_label;
    QLabel *z_label;
    QPushButton *goStationWithPointButton;
    QPlainTextEdit *wayPointText;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_3;
    QPushButton *goStationButton;
    QPushButton *removeButton;
    QPushButton *relocateButton;
    QPlainTextEdit *orientation_text;
    QLabel *label_5;

    void setupUi(QWidget *Station)
    {
        if (Station->objectName().isEmpty())
            Station->setObjectName(QString::fromUtf8("Station"));
        Station->resize(185, 270);
        layoutWidget = new QWidget(Station);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 171, 91));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);

        verticalLayout->addWidget(label);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFont(font);

        verticalLayout->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setFont(font);

        verticalLayout->addWidget(label_3);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font);

        verticalLayout->addWidget(label_4);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        station_label = new QLabel(layoutWidget);
        station_label->setObjectName(QString::fromUtf8("station_label"));
        station_label->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        verticalLayout_2->addWidget(station_label);

        x_label = new QLabel(layoutWidget);
        x_label->setObjectName(QString::fromUtf8("x_label"));
        x_label->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        verticalLayout_2->addWidget(x_label);

        y_label = new QLabel(layoutWidget);
        y_label->setObjectName(QString::fromUtf8("y_label"));
        y_label->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        verticalLayout_2->addWidget(y_label);

        z_label = new QLabel(layoutWidget);
        z_label->setObjectName(QString::fromUtf8("z_label"));
        z_label->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));

        verticalLayout_2->addWidget(z_label);


        horizontalLayout->addLayout(verticalLayout_2);

        goStationWithPointButton = new QPushButton(Station);
        goStationWithPointButton->setObjectName(QString::fromUtf8("goStationWithPointButton"));
        goStationWithPointButton->setGeometry(QRect(110, 240, 71, 27));
        wayPointText = new QPlainTextEdit(Station);
        wayPointText->setObjectName(QString::fromUtf8("wayPointText"));
        wayPointText->setGeometry(QRect(9, 237, 101, 31));
        layoutWidget1 = new QWidget(Station);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 140, 171, 95));
        verticalLayout_3 = new QVBoxLayout(layoutWidget1);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        goStationButton = new QPushButton(layoutWidget1);
        goStationButton->setObjectName(QString::fromUtf8("goStationButton"));

        verticalLayout_3->addWidget(goStationButton);

        removeButton = new QPushButton(layoutWidget1);
        removeButton->setObjectName(QString::fromUtf8("removeButton"));

        verticalLayout_3->addWidget(removeButton);

        relocateButton = new QPushButton(layoutWidget1);
        relocateButton->setObjectName(QString::fromUtf8("relocateButton"));

        verticalLayout_3->addWidget(relocateButton);

        orientation_text = new QPlainTextEdit(Station);
        orientation_text->setObjectName(QString::fromUtf8("orientation_text"));
        orientation_text->setGeometry(QRect(100, 105, 81, 30));
        label_5 = new QLabel(Station);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 105, 91, 30));
        QFont font1;
        font1.setBold(true);
        font1.setItalic(true);
        font1.setWeight(75);
        label_5->setFont(font1);

        retranslateUi(Station);

        QMetaObject::connectSlotsByName(Station);
    } // setupUi

    void retranslateUi(QWidget *Station)
    {
        Station->setWindowTitle(QApplication::translate("Station", "Station", nullptr));
        label->setText(QApplication::translate("Station", "Station", nullptr));
        label_2->setText(QApplication::translate("Station", "X:", nullptr));
        label_3->setText(QApplication::translate("Station", "Y:", nullptr));
        label_4->setText(QApplication::translate("Station", "Z:", nullptr));
        station_label->setText(QString());
        x_label->setText(QString());
        y_label->setText(QString());
        z_label->setText(QString());
        goStationWithPointButton->setText(QApplication::translate("Station", "GO!", nullptr));
        goStationButton->setText(QApplication::translate("Station", "GO!", nullptr));
        removeButton->setText(QApplication::translate("Station", "Remove", nullptr));
        relocateButton->setText(QApplication::translate("Station", "Relocate", nullptr));
        label_5->setText(QApplication::translate("Station", "Orientation:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Station: public Ui_Station {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STATION_H
