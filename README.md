# jane_ui
/****   very important here ****/
/* without clearing the qgraphicscene here,
 * Fxxxing memory leak will bully you */

void UIWindow::camimageDisplay(cv::Mat image)
{
  QImage imgIn= QImage((uchar*) image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  p_cam->clear(); // !!!!!!!!!!!!!!!! //
  p_cam->addPixmap(pixmap);
}
