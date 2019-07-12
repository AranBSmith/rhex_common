#ifndef RHEX_CONTROLLER_RHEX_CONTROLLER_BUEHLER
#define RHEX_CONTROLLER_RHEX_CONTROLLER_BUEHLER

#define _USE_MATH_DEFINES
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define PI 3.14159265
#define CTRL_SIZE 4
#define DOF 6

#define F 3
#define OFFSET 6.28

namespace rhex_controller {

    class RhexControllerBuehler {
    public:

        RhexControllerBuehler() {}

        RhexControllerBuehler(const std::vector<double>& ctrl)
        {
            set_parameters(ctrl);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            assert(ctrl.size() == CTRL_SIZE);

            _ctrl.resize(CTRL_SIZE, 0);
            for(size_t i = 0; i < CTRL_SIZE; ++i)
            {
                _ctrl[i] = ctrl[i];
            }

            _f = ctrl[0] * F;
            _period = 1 / _f;
            _duty_factor = ctrl[1];

            _duty_time = _duty_factor * _period;

            _stance_angle = ctrl[2] * PI;
            _stance_offset = ctrl[3] * OFFSET;

            _last_time = 0;
            _dt = 0.0;

            _phase.resize(DOF, 0);

            // set up weights between each of the legs, 1 for each except it_
            // [[0,1,1,1,1,1],[1,0,1,1,1,1],[1,1,0,1,1,1],[1,1,1,0,1,1],[1,1,1,1,0,1],[1,1,1,1,1,0]]
            _weights.resize(DOF, std::vector<double>(DOF, 0));
            for(size_t i = 0; i < DOF; ++i)
            {
                for(size_t j = 0; j < DOF; ++j)
                {
                    if (i != j)
                    {
                        _weights[i][j] = 1;
                    }
                }
            }

            _phase_bias.resize(DOF, std::vector<double>(DOF, 0));
            for(size_t i = 0; i < DOF; ++i)
            {
                for(size_t j = 0; j < DOF; ++j)
                {
                    if (i == 0 || i % 2 == 0)
                    {
                        if (j == 0 || j % 2 == 0)
                            _phase_bias[i][j] = 0;
                        else
                            _phase_bias[i][j] = PI;
                    }
                    else
                    {
                        if (j == 0 || j % 2 == 0)
                            _phase_bias[i][j] = -PI;
                        else
                            _phase_bias[i][j] = 0;
                    }
                }
            }

            _counter.resize(DOF, 0);

        }

        std::vector<double> pos(double t)
        {

            _dt = t - _last_time;
            _last_time = t;

            update();

            std::vector<double> output(DOF,0);

            for (size_t i = 0; i < DOF; ++i){
                double t = fmod(_phase[i], _period);
                if (t <= _duty_time)
                    output[i] = - _stance_angle / 2 + (_stance_angle / _duty_time) * t;

                else if(t > _duty_time && t <= _period)
                    output[i] = _stance_angle / 2 + ((2 * PI - _stance_angle)/(_period - _duty_time)) * (t - _duty_time);

                for (size_t j = 0; j < DOF; ++j)
                    output[i] += _weights[i][j] * (_phase[j] - _phase[i] - _phase_bias[i][j]);


                _counter[i] = floor(_phase[i] / _period);
                output[i] += _counter[i] * 2 * PI;

                for (size_t j = 0; j < DOF; ++j)
                    output[i] += _stance_offset;
            }

            std::cout << "phase: ";
            for (size_t i = 0; i < DOF; ++i)
                std::cout << _phase[i] << " ";
            std::cout<<std::endl;

            return output;
        }

        void update()
        {
            for (size_t i = 0; i < DOF; ++i)
                _phase[i] = _phase[i] + _dt;
        }

        const std::vector<double>& parameters() const
        {
            return _ctrl;
        }

    protected:
        double _f;
        double _period;
        double _duty_time;
        double _duty_factor;
        double _time;
        double _dt;
        double _last_time;
        double _stance_offset;
        double _stance_angle;

        std::vector<std::vector<double> > _phase_bias;
        std::vector<std::vector<double> > _weights;
        std::vector<int> _counter;
        std::vector<double> _ctrl;
        std::vector<double> _phase;
    };
}

#endif // RHEX_CONTROLLER_BUEHLER

