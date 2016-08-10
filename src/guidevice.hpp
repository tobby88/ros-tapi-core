#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include <QWidget>

class GuiDevice : public QWidget
{
  Q_OBJECT
public:
  explicit GuiDevice(QWidget* parent = 0);

  bool is_input_device;

  void paintEvent(QPaintEvent*);

private:
  int connectbox_size;
  int header_end;
  int footer_height;
  int footer_start;
  int line_height;
  int line_width;
  int line_start;
  int line_end;
  int items;
};

#endif // GuiDevice_H
