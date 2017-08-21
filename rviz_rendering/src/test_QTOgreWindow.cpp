/*

Place the include along with your other includes where you will be creating and/or calling the above QTOgreWindow:

*/


#include <QApplication>

#include "QTOgreWindow.h"

/*

In the method you are creating/calling a QTOgreWindow:

*/
int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  QTOgreWindow* ogreWindow = new QTOgreWindow();
  ogreWindow->show();

  return a.exec();
}