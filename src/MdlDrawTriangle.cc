#include <math.h>
#include <stdio.h>

#include "rtcore/ConfigTable.hh"
#include "rtcore/ModuleManager.hh"
#include "rtcore/Profiler.hh"
#include "MdlDrawTriangle.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedKinematics.hh"

using namespace rtcore;

extern QuadrupedKinematics::params_t createGo2Config();

#define DBGPRINT(...) printf(__VA_ARGS__)

MdlDrawTriangle::MdlDrawTriangle() : Module(WALKMODULE_NAME, 0, SINGLE_USER) {
    DBGPRINT("MdlDrawTriangle::MdlDrawTriangle\n");
};

MdlDrawTriangle::~MdlDrawTriangle() {
    DBGPRINT("MdlDrawTriangle::~MdlDrawTriangle\n");
};

void MdlDrawTriangle::init() {
    DBGPRINT("MdlDrawTriangle::init\n");

    for (int l = 0; l < 4; l++)
        _legs[l] = (MdlLegControl*)_mgr->findModule(LEGMODULE_NAME, l);
    
    _kinematics = new QuadrupedKinematics(createGo2Config());

    ConfigTable config;
    bool hasConfig = _mgr->getConfigTable("drawtriangle", config);

    if (hasConfig) {
        _tri_period = config.getDouble("period", _tri_period);
        _tri_xedge = config.getDouble("xedge", _tri_xedge);
        _tri_yedge = config.getDouble("yedge", _tri_yedge);

        ConfigArray origin;
        bool hasOrigin = config.getArray("origin", origin);
        if (hasOrigin) {
            _origin[0] = origin.getDoubleAt(0, -0.05);
            _origin[1] = origin.getDoubleAt(1, 0.12);
            _origin[2] = origin.getDoubleAt(2, -0.26);
        }
    }

    _fpos.push_back(Eigen::Vector3d(_tri_xedge / 2, _tri_yedge / 2, 0.0));
    _fpos.push_back(Eigen::Vector3d(_tri_xedge / 2, -_tri_yedge / 2, 0.0));
    _fpos.push_back(Eigen::Vector3d(-_tri_xedge / 2, 0.0, 0.0));

    for (int i = 0; i < 3; i++) _profiler[i] = new Profiler();
}

void MdlDrawTriangle::uninit() {DBGPRINT("MdlDrawTriangle::uninit\n");}

void MdlDrawTriangle::activate() {
    DBGPRINT("MdlDrawTriangle[%d]::activate\n", getIndex());

    for (int i = 0; i < 4; i++) _mgr->grabModule(_legs[i], this);

    _mark = _mgr->readTime();

    _state = _state_t::WAIT;
    _wait_entry();
}

void MdlDrawTriangle::deactivate() {
    DBGPRINT("MdlDrawTriangle::deactivate\n");

    for (int i = 0; i < 4; i++) _mgr->releaseModule(_legs[i], this);
}

bool MdlDrawTriangle::_wait_done(double t) {
    return (t - _mark > 3);
}

bool MdlDrawTriangle::_preplegs_done(double t) {
    return (t - _mark > _tri_period / 3);
}

// State methods
void MdlDrawTriangle::_wait_entry() {
  _resetTarget();
}
void MdlDrawTriangle::_wait_during() {
  _sendTarget();
}
void MdlDrawTriangle::_wait_exit() {}

void MdlDrawTriangle::_preplegs_entry() {
  _mark = _mgr->readTime();
  for (int i = 0; i < 3; i++) {
    _profiler[i]->clear();
    _profiler[i]->add(0, 0);
    _profiler[i]->add(_tri_period / 3, _fpos[0][i]);
  }
}

void MdlDrawTriangle::_preplegs_during() {
  _resetTarget();
  _computeProfile();
  _sendTarget();
}

void MdlDrawTriangle::_preplegs_exit() {}

void MdlDrawTriangle::_drawtriangle_entry() {
  _mark = _mgr->readTime();
  for (int i = 0; i < 3; i++) {
    _profiler[i]->clear();
    for (unsigned int j = 0; j < _fpos.size(); j++)
      _profiler[i]->add(j * _tri_period / _fpos.size(), _fpos[j][i]);
    _profiler[i]->setPeriod(_tri_period);
  }
}

void MdlDrawTriangle::_drawtriangle_during() {
  _resetTarget();
  _computeProfile();
  _sendTarget();
}
void MdlDrawTriangle::_drawtriangle_exit() {}

void MdlDrawTriangle::_computeProfile() {
  double t = _mgr->readTime();

  for (int i = 0; i < 3; i++) {
    Profiler::fval_t val;
    _profiler[i]->value(t - _mark, val);
    for (int j = 0; j < 4; j++) {
      _footpos[j][i] += val.v;
      _footvel[j][i] += val.d;
    }
  }
}

void MdlDrawTriangle::_sendTarget() {
  for (int i = 0; i < 4; i++) {
    _legs[i]->setTargetPosition(_footpos[i], _footvel[i]);
  }
}

void MdlDrawTriangle::_resetTarget() {
  // Reset the foot positions to the initial state
  for (int i = 0; i < 4; i++) {
    _footpos[i] = _kinematics->getKinematicParams().hip_positions.row(i).transpose();
    // Adjust abduction joint so that the feet are slightly outside and below the hip
    _footpos[i][0] += _origin[0];
    _footpos[i][1] += _origin[1] * (i % 2 == 0 ? 1 : -1);
    _footpos[i][2] += _origin[2];
  }
}

void MdlDrawTriangle::update() {
  double t = _mgr->readTime();

  for (int i = 0; i < 4; i++) _footvel[i] = Eigen::Vector3d::Zero();

  switch (_state) {
    case _state_t::WAIT:
      if (_wait_done(t)) {
        _state = _state_t::PREPLEGS;
        _wait_exit();
        _preplegs_entry();
        break;
      }
      _wait_during();
      break;
    case _state_t::PREPLEGS:
      if (_preplegs_done(t)) {
        _state = _state_t::DRAWTRIANGLE;
        _preplegs_exit();
        _drawtriangle_entry();
        break;
      }
      _preplegs_during();
      break;
    case _state_t::DRAWTRIANGLE:
      _drawtriangle_during();
      break;
    case _state_t::DONE:
      // Do nothing, we are done
      break;
  }

}