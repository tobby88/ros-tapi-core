#include "apideviceswrapper.hpp"
#include <QFont>
#include <QBrush>

ApiDevicesWrapper::ApiDevicesWrapper(QObject* parent)
    : QAbstractTableModel(parent)
{
}

ApiDevicesWrapper::~ApiDevicesWrapper() {}

int ApiDevicesWrapper::rowCount(const QModelIndex& /*parent*/) const
{
  return 2;
}

int ApiDevicesWrapper::columnCount(const QModelIndex& /*parent*/) const
{
  return 3;
}

QVariant ApiDevicesWrapper::data(const QModelIndex& index, int role) const
{
  int row = index.row();
  int col = index.column();

  switch (role)
  {
  case Qt::DisplayRole:
    if (row == 0 && col == 1)
      return QString("<--left");
    if (row == 1 && col == 1)
      return QString("right-->");

    return QString("Row%1, Column%2").arg(row + 1).arg(col + 1);
    break;
  case Qt::FontRole:
    if (row == 0 && col == 0) // change font only for cell(0,0)
    {
      QFont boldFont;
      boldFont.setBold(true);
      return boldFont;
    }
    break;
  case Qt::BackgroundRole:

    if (row == 1 && col == 2) // change background only for cell(1,2)
    {
      QBrush redBackground(Qt::red);
      return redBackground;
    }
    break;
  case Qt::TextAlignmentRole:

    if (row == 1 && col == 1) // change text alignment only for cell(1,1)
    {
      return Qt::AlignRight + Qt::AlignVCenter;
    }
    break;
  case Qt::CheckStateRole:

    if (row == 1 && col == 0) // add a checkbox to cell(1,0)
    {
      return Qt::Checked;
    }
  }
  return QVariant();
}