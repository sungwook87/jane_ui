#include "UIwindow.h"

namespace ui_server{
UIWindow::UIWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      m_RosThread(argc, argv)
{
    p_quitButton = new QPushButton(tr("&Quit"));
    p_takeoffButton = new QPushButton(tr("&TakeOff"));
    p_landingButton = new QPushButton(tr("&Landing"));
    p_elandingButton = new QPushButton(tr("&E-Landing"));
    p_missionstartButton = new QPushButton(tr("&Nonstop Mission"));
    p_missionpauseButton = new QPushButton(tr("&Mission pause"));
    p_missionuploadButton = new QPushButton(tr("&Mission upload"));
    p_camstartButton = new QPushButton(tr("&Record start"));
    p_camstopButton = new QPushButton(tr("&Record stop"));
    p_missionstopButton = new QPushButton(tr("&Mission stop"));
    p_sethome =new QPushButton(tr("&Set Home"));
    p_local2gps = new QPushButton(tr("&Local2GPS"));
    p_nextwaypoint = new QPushButton(tr("&Next WP"));
    p_previouswaypoint = new QPushButton(tr("&Previous WP"));
    p_killButton = new QPushButton(tr("&KILL"));

//    p_throttleup = new QPushButton();
//    p_throttledown = new QPushButton();
//    p_cwyaw = new QPushButton();
//    p_ccwyaw = new QPushButton();
    //p_forwardButton= new QPushButton();
    //p_backwardButton= new QPushButton();
    //p_leftButton= new QPushButton();
    //p_rightButton= new QPushButton();
    p_logoButton= new QPushButton();

    /** remote controller **/
/*
    QPalette palette = p_throttleup->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_throttleup->setAutoFillBackground(true);
    p_throttleup->setFlat(true);
    p_throttleup->setPalette(palette);
    p_throttleup->setIcon(QIcon(":/images/up.xpm"));
    p_throttleup->setIconSize(QSize(50, 50));

    palette = p_throttledown->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_throttledown->setAutoFillBackground(true);
    p_throttledown->setFlat(true);
    p_throttledown->setPalette(palette);
    p_throttledown->setIcon(QIcon(":/images/down.xpm"));
    p_throttledown->setIconSize(QSize(50, 50));

    palette = p_cwyaw->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_cwyaw->setAutoFillBackground(true);
    p_cwyaw->setFlat(true);
    p_cwyaw->setPalette(palette);
    p_cwyaw->setIcon(QIcon(":/images/right.xpm"));
    p_cwyaw->setIconSize(QSize(50, 50));

    palette = p_ccwyaw->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_ccwyaw->setAutoFillBackground(true);
    p_ccwyaw->setFlat(true);
    p_ccwyaw->setPalette(palette);
    p_ccwyaw->setIcon(QIcon(":/images/left.xpm"));
    p_ccwyaw->setIconSize(QSize(50, 50));

    palette = p_forwardButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_forwardButton->setAutoFillBackground(true);
    p_forwardButton->setFlat(true);
    p_forwardButton->setPalette(palette);
    p_forwardButton->setIcon(QIcon(":/images/up.xpm"));
    p_forwardButton->setIconSize(QSize(50, 50));

    palette = p_backwardButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_backwardButton->setAutoFillBackground(true);
    p_backwardButton->setFlat(true);
    p_backwardButton->setPalette(palette);
    p_backwardButton->setIcon(QIcon(":/images/down.xpm"));
    p_backwardButton->setIconSize(QSize(50, 50));

    palette = p_rightButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_rightButton->setAutoFillBackground(true);
    p_rightButton->setFlat(true);
    p_rightButton->setPalette(palette);
    p_rightButton->setIcon(QIcon(":/images/right.xpm"));
    p_rightButton->setIconSize(QSize(50, 50));

    palette = p_leftButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_leftButton->setAutoFillBackground(true);
    p_leftButton->setFlat(true);
    p_leftButton->setPalette(palette);
    p_leftButton->setIcon(QIcon(":/images/left.xpm"));
    p_leftButton->setIconSize(QSize(50, 50));
*/
    QPalette palette = p_logoButton->palette();
    palette.setColor(QPalette::Button,QColor(255,255,255));
    p_logoButton->setAutoFillBackground(true);
    p_logoButton->setFlat(true);
    p_logoButton->setPalette(palette);
    p_logoButton->setIcon(QIcon(":/images/customLogo.png"));
    p_logoButton->setIconSize(QSize(330, 110));

    /** Set up the Position Display **/
    leftLayout = new QVBoxLayout();
    p_xLayout = new QHBoxLayout();
    p_yLayout = new QHBoxLayout();
    p_zLayout = new QHBoxLayout();
    p_thetaLayout = new QHBoxLayout();

    p_estlatLayout = new QHBoxLayout();
    p_estlonLayout = new QHBoxLayout();
    p_estaltLayout = new QHBoxLayout();

    p_fixlatLayout = new QHBoxLayout();
    p_fixlonLayout = new QHBoxLayout();
    p_fixaltLayout = new QHBoxLayout();

    p_globallatLayout = new QHBoxLayout();
    p_globallonLayout = new QHBoxLayout();
    p_globalaltLayout = new QHBoxLayout();

    p_xvelLayout = new QHBoxLayout();
    p_yvelLayout = new QHBoxLayout();
    p_zvelLayout = new QHBoxLayout();

    p_globalheadLayout = new QHBoxLayout();
    p_imuLayout = new QHBoxLayout();
    p_armLayout = new QHBoxLayout();
    p_modeLayout = new QHBoxLayout();
    p_satelliteLayout = new QHBoxLayout();
    p_batteryLayout = new QHBoxLayout();
    p_flighttimeLayout = new QHBoxLayout();
    p_llogLayout = new QHBoxLayout();
   // p_llogLayout->setMargin(100);
    p_mapLayout = new QHBoxLayout();
    p_camLayout = new QHBoxLayout();


    p_satelliteLabel = new QLabel();
    p_satelliteLabel->setText(tr("GPS satellite:"));
    p_satelliteDisplay = new QLineEdit();
    p_satelliteDisplay->setText(tr("0.0"));

    p_batteryLabel = new QLabel();
    p_batteryLabel->setText(tr("Battery(%):"));
    p_batteryDisplay = new QLineEdit();
    p_batteryDisplay->setText(tr("0.0"));

    p_flighttimeLabel = new QLabel();
    p_flighttimeLabel->setText(tr("Time(s):"));
    p_flighttimeDisplay = new QLineEdit();
    p_flighttimeDisplay->setText(tr("0.0"));

    p_armLable = new QLabel();
    p_armLable->setText(tr("armed:"));
    p_armDisplay = new QLineEdit();
    p_armDisplay->setText(tr("-"));

    p_modeLable = new QLabel();
    p_modeLable->setText(tr("mode:"));
    p_modeDisplay = new QLineEdit();
    p_modeDisplay->setText(tr("-"));

    p_loglogLabel = new QLabel();
    p_loglogLabel->setText(tr(""));
    ploglog = new QTextBrowser();
    ploglog->setText(tr("     ***** Logs from pixhawk and log_data *****     "));
    ploglog->resize(300,100);


    // est gps
    p_estlatLabel = new QLabel();
    p_estlatLabel->setText(tr("Latitude(est):"));
    p_estlatDisplay = new QLineEdit();
    p_estlatDisplay->setText(tr("0.0"));
    p_estlonLabel = new QLabel();
    p_estlonLabel->setText(tr("Longitude(est):"));
    p_estlonDisplay = new QLineEdit();
    p_estlonDisplay->setText(tr("0.0"));
    p_estaltLabel = new QLabel();
    p_estaltLabel->setText(tr("Altitude(est):"));
    p_estaltDisplay = new QLineEdit();
    p_estaltDisplay->setText(tr("0.0"));
    // fix gps
    p_fixlatLabel = new QLabel();
    p_fixlatLabel->setText(tr("Latitude(fix):"));
    p_fixlatDisplay = new QLineEdit();
    p_fixlatDisplay->setText(tr("0.0"));
    p_fixlonLabel = new QLabel();
    p_fixlonLabel->setText(tr("Longitude(fix):"));
    p_fixlonDisplay = new QLineEdit();
    p_fixlonDisplay->setText(tr("0.0"));
    p_fixaltLabel = new QLabel();
    p_fixaltLabel->setText(tr("Altitude(fix):"));
    p_fixaltDisplay = new QLineEdit();
    p_fixaltDisplay->setText(tr("0.0"));
    // global gps
    p_globallatLabel = new QLabel();
    p_globallatLabel->setText(tr("Latitude(glb):"));
    p_globallatDisplay = new QLineEdit();
    p_globallatDisplay->setText(tr("0.0"));
    p_globallonLabel = new QLabel();
    p_globallonLabel->setText(tr("Longitude(glb):"));
    p_globallonDisplay = new QLineEdit();
    p_globallonDisplay->setText(tr("0.0"));
    p_globalaltLabel = new QLabel();
    p_globalaltLabel->setText(tr("Altitude(glb):"));
    p_globalaltDisplay = new QLineEdit();
    p_globalaltDisplay->setText(tr("0.0"));
    // body velocity
    p_xvelLabel = new QLabel();
    p_xvelLabel->setText(tr("x(m/s):"));
    p_xvelDisplay = new QLineEdit();
    p_xvelDisplay->setText(tr("0.0"));
    p_yvelLabel = new QLabel();
    p_yvelLabel->setText(tr("y(m/s):"));
    p_yvelDisplay = new QLineEdit();
    p_yvelDisplay->setText(tr("0.0"));
    p_zvelLabel = new QLabel();
    p_zvelLabel->setText(tr("z(m/s):"));
    p_zvelDisplay = new QLineEdit();
    p_zvelDisplay->setText(tr("0.0"));
    //gps heading
    p_globalheadLabel = new QLabel();
    p_globalheadLabel->setText(tr("GPS hdg:"));
    p_globalheadDisplay = new QLineEdit();
    p_globalheadDisplay->setText(tr("0.0"));

    p_imuLabel = new QLabel();
    p_imuLabel->setText(tr("IMU head.:"));
    p_imuDisplay = new QLineEdit();
    p_imuDisplay->setText(tr("0.0"));

    p_xLabel = new QLabel();
    p_xLabel->setText(tr("Local x:"));
    p_xDisplay = new QLineEdit();
    p_xDisplay->setText(tr("0.0"));

    p_yLabel = new QLabel();
    p_yLabel->setText(tr("Local y:"));
    p_yDisplay = new QLineEdit();
    p_yDisplay->setText(tr("0.0"));

    p_zLabel = new QLabel();
    p_zLabel->setText(tr("Local z: "));
    p_zDisplay = new QLineEdit();
    p_zDisplay->setText(tr("0.0"));

    p_thetaLabel = new QLabel();
    p_thetaLabel->setText(tr("Head.: "));
    p_thetaDisplay = new QLineEdit();
    p_thetaDisplay->setText(tr("0.0"));

    p_logoLayout = new QHBoxLayout();
    p_logoLayout->addSpacing(150);
    p_logoLayout->addWidget(p_logoButton);
    p_logoLayout->addSpacing(150);

    p_map = new QGraphicsScene();
    p_mapview = new QGraphicsView(p_map);
    p_cam = new QGraphicsScene();
    p_camview = new QGraphicsView(p_cam);
    //p_camview->sizePolicy();

    p_satelliteLayout->addWidget(p_satelliteLabel);
    p_satelliteLayout->addWidget(p_satelliteDisplay);
    p_satelliteLayout->addWidget(p_batteryLabel);
    p_satelliteLayout->addWidget(p_batteryDisplay);
    p_satelliteLayout->addWidget(p_flighttimeLabel);
    p_satelliteLayout->addWidget(p_flighttimeDisplay);
    p_satelliteLayout->addWidget(p_armLable);
    p_satelliteLayout->addWidget(p_armDisplay);
    p_satelliteLayout->addWidget(p_modeLable);
    p_satelliteLayout->addWidget(p_modeDisplay);

    p_globallatLayout->addWidget(p_globallatLabel);
    p_globallatLayout->addWidget(p_globallatDisplay);
    p_globallatLayout->addWidget(p_globallonLabel);
    p_globallatLayout->addWidget(p_globallonDisplay);
    p_globallatLayout->addWidget(p_globalaltLabel);
    p_globallatLayout->addWidget(p_globalaltDisplay);
    p_globallatLayout->addWidget(p_globalheadLabel);
    p_globallatLayout->addWidget(p_globalheadDisplay);

    p_estlatLayout->addWidget(p_estlatLabel);
    p_estlatLayout->addWidget(p_estlatDisplay);
    p_estlatLayout->addWidget(p_estlonLabel);
    p_estlatLayout->addWidget(p_estlonDisplay);
    p_estlatLayout->addWidget(p_estaltLabel);
    p_estlatLayout->addWidget(p_estaltDisplay);
    p_estlatLayout->addWidget(p_imuLabel);
    p_estlatLayout->addWidget(p_imuDisplay);

    p_llogLayout->addWidget(ploglog);
    p_mapLayout->addWidget(p_mapview);
    p_mapLayout->addWidget(p_camview);

    p_xLayout->addWidget(p_xLabel);
    p_xLayout->addWidget(p_xDisplay);
    p_xLayout->addWidget(p_yLabel);
    p_xLayout->addWidget(p_yDisplay);
    p_xLayout->addWidget(p_zLabel);
    p_xLayout->addWidget(p_zDisplay);
    p_xLayout->addWidget(p_thetaLabel);
    p_xLayout->addWidget(p_thetaDisplay);

    p_xLayout->addWidget(p_xvelLabel);
    p_xLayout->addWidget(p_xvelDisplay);
    p_xLayout->addWidget(p_yvelLabel);
    p_xLayout->addWidget(p_yvelDisplay);
    p_xLayout->addWidget(p_zvelLabel);
    p_xLayout->addWidget(p_zvelDisplay);

    leftLayout->addLayout(p_satelliteLayout);
    leftLayout->addLayout(p_globallatLayout);
  //  leftLayout->addLayout(p_estlatLayout);
  //  leftLayout->addLayout(p_xLayout);
  //  leftLayout->addLayout(p_camLayout);
    leftLayout->addLayout(p_mapLayout);
    leftLayout->addLayout(p_logoLayout);



    /** Set up the Layouts **/

    rightLayout = new QVBoxLayout();
    //rightLayout->setSizeConstraint(SetMinimumSize);
    layout = new QHBoxLayout();
    layout2 = new QHBoxLayout();
    layout3 = new QHBoxLayout();
    layout4 = new QHBoxLayout();
    layout5 = new QHBoxLayout();
    layout6 = new QHBoxLayout();
    layout7 = new QHBoxLayout();
    layout8 = new QHBoxLayout();
    layout9 = new QHBoxLayout();
    layout10 = new QHBoxLayout();



    mainLayout = new QHBoxLayout();

    layout->addWidget(p_takeoffButton);
    layout->addWidget(p_landingButton);
    layout->addWidget(p_elandingButton);

    layout2->addWidget(p_sethome);
    layout2->addWidget(p_local2gps);

    layout3->addWidget(p_missionuploadButton);
    layout3->addWidget(p_missionstartButton);

    layout4->addWidget(p_missionpauseButton);
    layout4->addWidget(p_missionstopButton);

    layout5->addWidget(p_previouswaypoint);
    layout5->addWidget(p_nextwaypoint);

    layout6->addWidget(p_camstartButton);
    layout6->addWidget(p_camstopButton);

//    layout7->addSpacing(50);
//    layout7->addWidget(p_throttleup);
//    layout7->addSpacing(100);
//    layout7->addWidget(p_forwardButton);
//    layout7->addSpacing(50);

//    layout8->addWidget(p_ccwyaw);
//    layout8->addWidget(p_cwyaw);
//    layout8->addWidget(p_leftButton);
//    layout8->addWidget(p_rightButton);

//    layout9->addSpacing(50);
//    layout9->addWidget(p_throttledown);
//    layout9->addSpacing(100);
//    layout9->addWidget(p_backwardButton);
//    layout9->addSpacing(50);
    layout9->addWidget(p_killButton);
    layout10->addWidget(p_quitButton);

    rightLayout->addLayout(layout);
    rightLayout->addLayout(layout2);
    rightLayout->addLayout(layout3);
    rightLayout->addLayout(layout4);
    rightLayout->addLayout(layout5);
    rightLayout->addLayout(layout6);
    rightLayout->addLayout(p_llogLayout);

//    rightLayout->addLayout(layout7);
//    rightLayout->addLayout(layout8);
    rightLayout->addLayout(layout9);
    rightLayout->addLayout(layout10);

    mainLayout->addLayout(leftLayout);
   // mainLayout->addLayout(p_mapLayout);
    mainLayout->addLayout(rightLayout);
    setLayout(mainLayout);

    show();

    setWindowTitle(tr("Jane is FREE"));

    connect(p_quitButton,           &QPushButton::clicked, this, &UIWindow::muin_quit);
    connect(p_takeoffButton,        &QPushButton::clicked, this, &UIWindow::muin_takeoff);
    connect(p_landingButton,        &QPushButton::clicked, this, &UIWindow::muin_landing);
    connect(p_elandingButton,       &QPushButton::clicked, this, &UIWindow::muin_elanding);
    connect(p_missionstartButton,   &QPushButton::clicked, this, &UIWindow::muin_nonstopmission);
    connect(p_missionpauseButton,   &QPushButton::clicked, this, &UIWindow::muin_missionpause);
    connect(p_local2gps,            &QPushButton::clicked, this, &UIWindow::muin_local2gps);
    connect(p_nextwaypoint,         &QPushButton::clicked, this, &UIWindow::muin_nextmission);
    connect(p_previouswaypoint,     &QPushButton::clicked, this, &UIWindow::muin_prevmission);
    connect(p_missionuploadButton,  &QPushButton::clicked, this, &UIWindow::muin_missionupload);
    connect(p_camstartButton,       &QPushButton::clicked, this, &UIWindow::muin_camstart);
    connect(p_camstopButton,        &QPushButton::clicked, this, &UIWindow::muin_camstop);
    connect(p_killButton,        &QPushButton::clicked, this, &UIWindow::muin_kill);

    connect(&m_RosThread,         &RosThread::localpose, this, &UIWindow::updatePoseDisplay);
    connect(&m_RosThread,         &RosThread::gpscount, this, &UIWindow::satelliteDisplay);
    connect(&m_RosThread,         &RosThread::estGPS, this, &UIWindow::estGPSDisplay);
    connect(&m_RosThread,         &RosThread::imu, this, &UIWindow::imuDisplay);
    connect(&m_RosThread,         &RosThread::battery, this, &UIWindow::batDisplay);
    connect(&m_RosThread,         &RosThread::globalGPS, this, &UIWindow::globalgpsDisplay);
    connect(&m_RosThread,         &RosThread::bodyvel, this, &UIWindow::bodyvelDisplay);
    connect(&m_RosThread,         &RosThread::compass, this, &UIWindow::gpscompassDisplay);

    qRegisterMetaType<std::string>("std::string");
    connect(&m_RosThread,         &RosThread::state, this, &UIWindow::stateDisplay);
    connect(&m_RosThread,         &RosThread::status, this, &UIWindow::logDisplay);
    connect(&m_RosThread,         &RosThread::logdata, this, &UIWindow::logDisplay);
    qRegisterMetaType< cv::Mat >("cv::Mat");
    connect(&m_RosThread,         &RosThread::mapimage, this, &UIWindow::mapimageDisplay);
    qRegisterMetaType< cv::Mat >("cv::Mat");
    connect(&m_RosThread,         &RosThread::camimage, this, &UIWindow::camimageDisplay);


    m_RosThread.init();
}//end constructor

//void UIWindow::halt(){ m_RosThread.SetSpeed(0, 0); }

void UIWindow::muin_takeoff(){m_RosThread.fn_take_off();}
void UIWindow::muin_landing(){m_RosThread.fn_landing();}
void UIWindow::muin_elanding(){m_RosThread.fn_emergency_landing();}
void UIWindow::muin_missionpause(){m_RosThread.fn_pause_mission();}
void UIWindow::muin_local2gps(){m_RosThread.fn_local2gps();}
void UIWindow::muin_nextmission(){m_RosThread.fn_next_mission();}
void UIWindow::muin_prevmission(){m_RosThread.fn_prev_mission();}
void UIWindow::muin_missionupload(){m_RosThread.fn_upload_mission();}
void UIWindow::muin_nonstopmission(){m_RosThread.fn_nonstop_mission();}
void UIWindow::muin_quit()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "Hey!", "want to Quit?", QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::Yes) {
    qDebug() << "Bye~ jane";
    close();
  } else {
    qDebug() << "Whooo!";
  }
}
void UIWindow::muin_camstart(){m_RosThread.fn_record_start();}
void UIWindow::muin_camstop(){m_RosThread.fn_record_stop();}
void UIWindow::muin_kill()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::critical(this, "Hey!", "KILL ?", QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::Yes) {
    qDebug() << "T_T";
    m_RosThread.fn_kill();
  } else {
    qDebug() << "^_^!";
  }
}


void UIWindow::satelliteDisplay(unsigned int count)
{
  QString cnt;
  cnt.setNum(count);
  p_satelliteDisplay->setText(cnt);
}

void UIWindow::batDisplay(double bat)
{
  QString qbat;
  qbat.setNum(bat);
  p_batteryDisplay->setText(qbat);
}

void UIWindow::imuDisplay(double heading)
{
  QString qheading;
  qheading.setNum(heading);
  p_imuDisplay->setText(qheading);
}


void UIWindow::logDisplay(std::string log)
{
//  QString qlog;
//  qlog.sprintf(log.c_str());

//  p_logDisplay->setText(qlog);

 // m_logcnt = 0;
//  QString qlog;
//  QStringList qllog;
//  qlog.sprintf(log.c_str());
//  qllog.
//  qllog.sprintf(log.c_str());
//  row = m_qlw->currentRow();
//  m_qlwi->setText(qlog);
//  //m_qlw->insertItem(m_logcnt,m_qlwi);
//  m_qlw->addItems(qllog);
//  m_qlw->setCurrentRow(m_logcnt++);
//  std::cout<<m_logcnt<<std::endl;
  QString qlog;
  int i=0;
  qlog.sprintf(log.c_str());
  ploglog->append(qlog);
}

void UIWindow::stateDisplay(std::string arm, std::string mode)
{
  QString qarm, qmode;
  qarm.sprintf(arm.c_str());
  p_armDisplay->setText(qarm);
  qmode.sprintf(mode.c_str());
  p_modeDisplay->setText(qmode);
}

void UIWindow::estGPSDisplay(double lon, double lat, double alt)
{
  QString qlon, qlat, qalt;
  qlon.setNum(lon);
  qlat.setNum(lat);
  qalt.setNum(alt);

  p_estlatDisplay->setText(qlon);
  p_estlonDisplay->setText(qlat);
  p_estaltDisplay->setText(qalt);
}

void UIWindow::updatePoseDisplay(double x, double y, double z, double theta)
{
  QString xPose, yPose, zPose, thetaPose;
  xPose.setNum(x);
  yPose.setNum(y);
  zPose.setNum(z);
  thetaPose.setNum(theta);

  p_xDisplay->setText(xPose);
  p_yDisplay->setText(yPose);
  p_zDisplay->setText(zPose);
  p_thetaDisplay->setText(thetaPose);

}//update the display.

void UIWindow::globalgpsDisplay(double lon, double lat, double alt)
{
  QString qlon, qlat, qalt;
  qlon.setNum(lon);
  qlat.setNum(lat);
  qalt.setNum(alt);

  p_globallonDisplay->setText(qlon);
  p_globallatDisplay->setText(qlat);
  p_globalaltDisplay->setText(qalt);
}

void UIWindow::gpscompassDisplay(double heading)
{
  QString qhdg;
  qhdg.setNum(heading);
  p_globalheadDisplay->setText(qhdg);
}

void UIWindow::bodyvelDisplay(double x, double y, double z)
{
  QString vx, vy, vz;
  vx.setNum(x);
  vy.setNum(y);
  vz.setNum(z);

  p_xvelDisplay->setText(vx);
  p_yvelDisplay->setText(vy);
  p_zvelDisplay->setText(vz);
}

void UIWindow::mapimageDisplay(cv::Mat image)
{
  QImage imgIn= QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);

  /****   very important here ****/
  /* without clearing the qgraphicscene here,
   * Fxxxing memory leak will bully you */
  p_cam->clear();
  p_map->addPixmap(pixmap);

 // std::cout<<"hi"<<std::endl;

}

void UIWindow::camimageDisplay(cv::Mat image)
{
  QImage imgIn= QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  /****   very important here ****/
  /* without clearing the qgraphicscene here,
   * Fxxxing memory leak will bully you */
  p_cam->clear();
  p_cam->addPixmap(pixmap);
}

}//end namespace

