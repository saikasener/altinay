#ifndef STATION_H
#define STATION_H

#include <QWidget>
#include "station_tool.h"
#include "way_point_tool.h"

extern moobot_ui::StationTool * station_tool;

namespace Ui {
class Station;
}

class Station : public QWidget
{
    Q_OBJECT

public:
    explicit Station(QWidget *parent = 0);
    ~Station();
    void make_station(int station_num,float x,float y,float z);
    void update_station(float x,float y,float z);
    Ui::Station *ui;
    int station_num;
    float x,y,z;
    bool isGoWithPointPressed;
    double orientation = 0.0;

 signals:
    void removeSignal(const int num);
    void relocateSignal(const int num);
    void pointSignal(const std::string go_point, const float x, const float y, const float z, const int station_num );

public slots:
    void enableRemove(bool isRelocateDone);
    void goWithPointErrorSlot(QString error,int station_num);
    void goWithPointIsDoneSlot(bool done,int station_num);
private slots:
    void on_relocateButton_clicked();
    void on_removeButton_clicked();
    void on_goStationButton_clicked();

    void on_goStationWithPointButton_pressed();

    void on_orientation_text_textChanged();

private:

    bool isRemovePressed,isRelocatePressed;
};

#endif // STATION_H
