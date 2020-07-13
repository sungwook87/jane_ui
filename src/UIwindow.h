#ifndef UI_WINDOW_H
#define UI_WINDOW_H
#include <QtWidgets>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextStream>
#include <QLineEdit>
#include <QPalette>
#include <QIcon>
#include "rosthread.h"
#include <iostream>
#include <QListView>
#include <QPixmap>
#include <QGraphicsView>
#include <QGraphicsScene>

namespace ui_server{
#define PI 3.14159265359
//Q_DECLARE_METATYPE (std::string)

class UIWindow : public QWidget
{
    Q_OBJECT

public:
    UIWindow(int argc, char** argv, QWidget * parent = 0);

    Q_SLOT void updatePoseDisplay(double x, double y, double z, double theta);
    Q_SLOT void satelliteDisplay(unsigned int count);
    Q_SLOT void estGPSDisplay(double lon, double lat, double alt);
    Q_SLOT void imuDisplay(double heading);
    Q_SLOT void logDisplay(std::string log);
    Q_SLOT void batDisplay(double bat);
    Q_SLOT void stateDisplay(std::string arm, std::string mode);

  //  Q_SLOT void fixgpsDisplay(double lon, double lat, double alt);
    Q_SLOT void bodyvelDisplay(double x, double y, double z);
    Q_SLOT void globalgpsDisplay(double lon, double lat, double alt);
    Q_SLOT void gpscompassDisplay(double heading);
    Q_SLOT void mapimageDisplay(cv::Mat image);
    Q_SLOT void camimageDisplay(cv::Mat image);




  //  qRegisterMetaType<std::std>


private:
    Q_SLOT void muin_takeoff();
    Q_SLOT void muin_landing();
    Q_SLOT void muin_elanding();
    Q_SLOT void muin_missionpause();
    Q_SLOT void muin_camstart();
    Q_SLOT void muin_camstop();
    Q_SLOT void muin_local2gps();
    Q_SLOT void muin_nextmission();
    Q_SLOT void muin_prevmission();
    Q_SLOT void muin_missionupload();
    Q_SLOT void muin_nonstopmission();
    Q_SLOT void muin_kill();
    Q_SLOT void muin_sethome();
    Q_SLOT void muin_rth();

    void muin_quit();

   // Q_SLOT void halt();
    QPushButton *p_killButton;
    QPushButton *p_takeoffButton;
    QPushButton *p_landingButton;
    QPushButton *p_elandingButton;
    QPushButton *p_missionstartButton;
    QPushButton *p_missionpauseButton;
    QPushButton *p_missionstopButton;
    QPushButton *p_missionuploadButton;
    QPushButton *p_sethome;
    QPushButton *p_local2gps;
    QPushButton *p_nextwaypoint;
    QPushButton *p_previouswaypoint;
    QPushButton *p_camstartButton;
    QPushButton *p_camstopButton;
    QPushButton *p_quitButton;
    QPushButton *p_rthButton;


    QPushButton *p_throttleup;
    QPushButton *p_throttledown;
    QPushButton *p_cwyaw;
    QPushButton *p_ccwyaw;
//    QPushButton *p_forwardButton;
//    QPushButton *p_backwardButton;
//    QPushButton *p_leftButton;
//    QPushButton *p_rightButton;
    QPushButton *p_logoButton;

    QTextBrowser *ploglog;

  //  std::string log;



    QVBoxLayout *rightLayout;
    QHBoxLayout *layout;
    QHBoxLayout *layout2;
    QHBoxLayout *layout3;
    QHBoxLayout *layout4;
    QHBoxLayout *layout5;
    QHBoxLayout *layout6;
    QHBoxLayout *layout7;
    QHBoxLayout *layout10;
    QHBoxLayout *layout8;
    QHBoxLayout *layout9;

    QVBoxLayout *leftLayout;

    // local pose
    QHBoxLayout *p_xLayout;
    QHBoxLayout *p_yLayout;
    QHBoxLayout *p_zLayout;
    QHBoxLayout *p_thetaLayout;
    QLabel *p_xLabel;
    QLabel *p_yLabel;
    QLabel *p_zLabel;
    QLabel *p_thetaLabel;
    QLineEdit *p_xDisplay;
    QLineEdit *p_yDisplay;
    QLineEdit *p_zDisplay;
    QLineEdit *p_thetaDisplay;
    // est. GPS
    QHBoxLayout *p_estlatLayout;
    QHBoxLayout *p_estlonLayout;
    QHBoxLayout *p_estaltLayout;
    QLabel *p_estlatLabel;
    QLabel *p_estlonLabel;
    QLabel *p_estaltLabel;
    QLineEdit *p_estlatDisplay;
    QLineEdit *p_estlonDisplay;
    QLineEdit *p_estaltDisplay;
    // fix GPS
    QHBoxLayout *p_fixlatLayout;
    QHBoxLayout *p_fixlonLayout;
    QHBoxLayout *p_fixaltLayout;
    QLabel *p_fixlatLabel;
    QLabel *p_fixlonLabel;
    QLabel *p_fixaltLabel;
    QLineEdit *p_fixlatDisplay;
    QLineEdit *p_fixlonDisplay;
    QLineEdit *p_fixaltDisplay;
    // global GPS
    QHBoxLayout *p_globallatLayout;
    QHBoxLayout *p_globallonLayout;
    QHBoxLayout *p_globalaltLayout;
    QLabel *p_globallatLabel;
    QLabel *p_globallonLabel;
    QLabel *p_globalaltLabel;
    QLineEdit *p_globallatDisplay;
    QLineEdit *p_globallonDisplay;
    QLineEdit *p_globalaltDisplay;
    // body vel
    QHBoxLayout *p_xvelLayout;
    QHBoxLayout *p_yvelLayout;
    QHBoxLayout *p_zvelLayout;
    QLabel *p_xvelLabel;
    QLabel *p_yvelLabel;
    QLabel *p_zvelLabel;
    QLineEdit *p_xvelDisplay;
    QLineEdit *p_yvelDisplay;
    QLineEdit *p_zvelDisplay;
    // global heading
    QHBoxLayout *p_globalheadLayout;
    QLabel *p_globalheadLabel;
    QLineEdit *p_globalheadDisplay;

    QHBoxLayout *p_satelliteLayout;
    QHBoxLayout *p_batteryLayout;
    QHBoxLayout *p_flighttimeLayout;
    QHBoxLayout *p_armLayout;
    QHBoxLayout *p_modeLayout;
    QHBoxLayout *p_logLayout;
    QHBoxLayout *p_llogLayout;
    QHBoxLayout *p_imuLayout;
    QHBoxLayout *p_logoLayout;
    QHBoxLayout *p_mapLayout;
    QLabel *p_imuLabel;
    QLabel *p_satelliteLabel;
    QLabel *p_batteryLabel;
    QLabel *p_flighttimeLabel;
    QLabel *p_armLable;
    QLabel *p_modeLable;
    QLabel *p_loglogLabel;
    QLabel *p_scanLabel;

    QLineEdit *p_imuDisplay;
    QLineEdit *p_armDisplay;
    QLineEdit *p_modeDisplay;
    QLineEdit *p_satelliteDisplay;
    QLineEdit *p_batteryDisplay;
    QLineEdit *p_flighttimeDisplay;

    QGraphicsScene *p_map;
    QGraphicsView *p_mapview;
    QGraphicsScene *p_cam;
    QGraphicsView *p_camview;
    QHBoxLayout *p_camLayout;

    QVBoxLayout *bottomLayout;
    QPixmap logo;

    QHBoxLayout *mainLayout;
    QPushButton *closeButton;

    RosThread m_RosThread;
};//end class ControlWindow
}//end namespace
#endif

