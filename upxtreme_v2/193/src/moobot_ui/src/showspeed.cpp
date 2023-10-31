#include "showspeed.h"
#include "ui_showspeed.h"



ShowSpeed::ShowSpeed(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ShowSpeed)
{
    ui->setupUi(this);
}

ShowSpeed::~ShowSpeed()
{
    delete ui;
}

double ShowSpeed::linearSpeed_act=0;
double ShowSpeed::angularSpeed_act= 0;
double ShowSpeed::linearSpeed_des= 0;
double ShowSpeed::angularSpeed_des= 0;
double ShowSpeed::linearSpeed_max = 0;
double ShowSpeed::linearSpeed_min = 0;
double ShowSpeed::angularSpeed_max = 0;
double ShowSpeed::angularSpeed_min = 0;
int ShowSpeed::precision = 4;

void ShowSpeed::ShowSpeedValues(){


    ui->angularSpeedAct->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed_act, 'f', precision)));
    ui->angularSpeedDes->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed_des, 'f', precision)));

    ui->linearSpeedAct->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed_act, 'f', precision)));
    ui->linearSpeedDes->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed_des, 'f', precision)));

}
void ShowSpeed::updateMaxSpeed(){
    ui->angularSpeedMax->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed_max, 'f', precision)));
    ui->linearSpeedMax->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed_max, 'f', precision)));


}
void ShowSpeed::updateMinSpeed(){
    ui->angularSpeedMin->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(angularSpeed_min, 'f', precision)));
    ui->linearSpeedMin->setText(QString("<span style=\" font-size:13pt; font-weight:600;\">%1</span>").arg(QString::number(linearSpeed_min, 'f', precision)));
}


/*void ShowSpeed::on_precision_box_editingFinished()
{
    precision = ui->precision_box->value();
    this->ShowSpeedValues();
    this->updateMaxSpeed();
    this->updateMinSpeed();
}*/
void ShowSpeed::on_speedChange(){
    this->ShowSpeedValues();

}

void ShowSpeed::on_maxSpeedChange(){
    this->updateMaxSpeed();
}
void ShowSpeed::on_minSpeedChange(){
    this->updateMinSpeed();
}
