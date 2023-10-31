#ifndef SHOWSPEED_H
#define SHOWSPEED_H

#include <QWidget>
//#define deg 57.295778
#define deg 57.3

namespace Ui {
class ShowSpeed;
}

class ShowSpeed : public QWidget
{
    Q_OBJECT

public:
    explicit ShowSpeed(QWidget *parent = 0);
    ~ShowSpeed();


    static double linearSpeed_act;
    static double angularSpeed_act;
    static double linearSpeed_des;
    static double angularSpeed_des;
    static double angularSpeed_max;
    static double angularSpeed_min;
    static double linearSpeed_max;
    static double linearSpeed_min;
    static int precision;



    void ShowSpeedValues();
    void updateMaxSpeed();
    void updateMinSpeed();

public slots:
    void on_precision_box_editingFinished();

    void on_speedChange();
    void on_maxSpeedChange();
    void on_minSpeedChange();
private:
    Ui::ShowSpeed *ui;
};
#endif // SHOWSPEED_H
