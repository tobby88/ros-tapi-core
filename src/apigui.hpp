#ifndef APIGUI_HPP
#define APIGUI_HPP

#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"
#include <QMainWindow>
#include <QTimer>
#include <QVBoxLayout>
#include <vector>

namespace Ui
{
class ApiGui;
}

class ApiGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit ApiGui(Api* api, QWidget* parent = 0);
  ~ApiGui();

private:
  Ui::ApiGui* ui;
  Api* api;
  QTimer* timer;
  void addDevice(Device* device);
  unsigned int temp2;
  QVBoxLayout* layoutSender;
  QVBoxLayout* layoutReceiver;
  QVBoxLayout* layoutConnections;
  vector<GuiDevice*> receiverGuiDevices;
  vector<GuiDevice*> senderGuiDevices;

private slots:
  void checkApiForUpdate();
};

#endif // APIGUI_HPP
