#ifndef WALKMODULE_HH
#define WALKMODULE_HH

#include <vector>
#include "rtcore/Module.hh"

#include "Eigen/Dense"

class MdlLegControl;
class QuadrupedKinematics;

namespace rtcore
{
    class Profiler;
}

#define WALKMODULE_NAME "MdlDrawTriangle"

class MdlDrawTriangle : public rtcore::Module
{
public:
    MdlDrawTriangle();
    ~MdlDrawTriangle();

    void init();
    void uninit();
    void activate();
    void deactivate();
    void update();

private:
    enum class _state_t
    {
        WAIT,
        PREPLEGS,
        DRAWTRIANGLE,
        DONE
    };
    _state_t _state = _state_t::WAIT;

    // Event Methods
    bool _wait_done(double t);
    bool _preplegs_done(double t);

    // State Methods
    void _wait_entry();
    void _wait_during();
    void _wait_exit();

    void _preplegs_entry();
    void _preplegs_during();
    void _preplegs_exit();

    void _drawtriangle_entry();
    void _drawtriangle_during();
    void _drawtriangle_exit();

    Eigen::Vector3d _footpos[4];
    Eigen::Vector3d _footvel[4];
    void _resetTarget();
    void _computeProfile();
    void _sendTarget();

    MdlLegControl *_legs[4];
    QuadrupedKinematics *_kinematics = nullptr;
    rtcore::Profiler *_profiler[3] = {nullptr,nullptr,nullptr};
    double _mark = 0.0;

    double _origin[3] = {-0.05, 0.12, -0.26};
    double _tri_period = 10;
    double _tri_xedge = 0.15;
    double _tri_yedge = 0.15;

    std::vector<Eigen::Vector3d> _fpos;

};
#endif