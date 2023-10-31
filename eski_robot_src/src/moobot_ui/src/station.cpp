#include "station.h"
#include "ui_station.h"


Station::Station(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Station)
{
    ui->setupUi(this);

}

Station::~Station()
{
    delete ui;
}
void Station::make_station(int station_num,float x,float y,float z){
    ui->station_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(station_num));
    ui->x_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(x, 'f', 4)));
    ui->y_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(y, 'f', 4)));
    ui->z_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(z, 'f', 4)));

    this->station_num = station_num;

    this->x = x;
    this->y = y;
    this->z = z;

    isRelocatePressed = isGoWithPointPressed = false;

}
void Station::update_station(float x,float y,float z){
    ui->x_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(x, 'f', 4)));
    ui->y_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(y, 'f', 4)));
    ui->z_label->setText(QString("<span style=\" font-size:14pt; font-weight:600;\">%1</span>").arg(QString::number(z, 'f', 4)));

    this->station_num = station_num;

    this->x = x;
    this->y = y;
    this->z = z;
}

void Station::on_goStationButton_clicked()
{
    moobot_ui::StationTool* station_tool = new moobot_ui::StationTool();

    station_tool->onPoseSet(x,y,orientation);
}

void Station::on_removeButton_clicked()
{
    emit removeSignal(this->station_num);
    

}

void Station::on_relocateButton_clicked()
{
    this->ui->removeButton->setEnabled(false);
    emit relocateSignal(this->station_num);


}
void Station::enableRemove(bool isRelocateDone){

   this->ui->removeButton->setEnabled(isRelocateDone);

}

void Station::on_goStationWithPointButton_pressed()
{

    if(!isGoWithPointPressed){
        emit pointSignal(this->ui->wayPointText->toPlainText().toStdString(),x,y,z,station_num);

    }





}
void Station::goWithPointErrorSlot(QString error,int station_num){
    if(this->station_num == station_num )
        this->ui->wayPointText->setPlainText(error);
}
void Station::goWithPointIsDoneSlot(bool done,int station_num){
    if(this->station_num == station_num || station_num == 0){
        this->ui->goStationWithPointButton->setEnabled(done);
        isGoWithPointPressed = !done;
    }



}

void Station::on_orientation_text_textChanged()
{
    orientation = ui->orientation_text->toPlainText().toDouble();
}
