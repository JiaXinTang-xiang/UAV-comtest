// Limit only the desktop view. The server's optical flow camera runs separately.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/common/Console.hh>
#include <QTimer>
#include <cmath>

namespace gazebo
{
class VmGui : public GUIPlugin
{
public:
  void Load(sdf::ElementPtr sdf) override
  {
    const double requested = sdf->HasElement("fps") ? sdf->Get<double>("fps") : 15.0;
    const double fps = std::isfinite(requested) && requested >= 5 && requested <= 30
                         ? requested : 15.0;
    setFixedSize(0, 0);
    auto *timer = new QTimer(this);
    // GUI plugins can load before UserCamera exists. Retry on the GUI thread,
    // then stop the timer after setting both the camera and Qt redraw interval.
    connect(timer, &QTimer::timeout, this, [timer, fps]() {
      if (!gui::get_active_camera())
        return;
      gui::Events::setRenderRate(fps);
      gzmsg << "VM GUI render limit: " << fps << " FPS\n";
      timer->stop();
    });
    timer->start(250);
  }
};
GZ_REGISTER_GUI_PLUGIN(VmGui)
}
