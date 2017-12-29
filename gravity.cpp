/*
 * gravity.cpp
 *
 *  Created on: Dec 1, 2017
 *      Author: n69136
 */

#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <array>
#include <deque>
#include <string>
#include <sstream>
#include <memory>

#include "ppm.h"

#include "geometry.h"

namespace gravity {

    typedef long double D;

    class point3d {
    public:
        std::array<D, 3> coord_;

    public:
        point3d() :
                coord_()
        {
        }

        point3d(D x, D y, D z) :
                coord_( { x, y, z })
        {
        }

        D& x()
        {
            return coord_[0];
        }
        D x() const
        {
            return coord_[0];
        }
        D& y()
        {
            return coord_[1];
        }
        D y() const
        {
            return coord_[1];
        }
        D& z()
        {
            return coord_[2];
        }
        D z() const
        {
            return coord_[2];
        }

        static D distance(const point3d& p1, const point3d& p2);
        static D distance2(const point3d& p1, const point3d& p2);
        D distance(const point3d& p2) const
        {
            return distance(*this, p2);
        }

        void rotate_x(D angle)
        {
            D cosa = std::cos(angle);
            D sina = std::sin(angle);
            auto y = coord_[1] * cosa - coord_[2] * sina;
            auto z = coord_[1] * sina + coord_[2] * cosa;
            coord_[1] = y;
            coord_[2] = z;
        }
        void rotate_y(D angle)
        {
            D cosa = std::cos(angle);
            D sina = std::sin(angle);
            auto z = coord_[2] * cosa - coord_[0] * sina;
            auto x = coord_[2] * sina + coord_[0] * cosa;
            coord_[0] = x;
            coord_[2] = z;
        }
        void rotate_z(D angle)
        {
            D cosa = std::cos(angle);
            D sina = std::sin(angle);
            auto x = coord_[0] * cosa - coord_[1] * sina;
            auto y = coord_[0] * sina + coord_[1] * cosa;
            coord_[0] = x;
            coord_[1] = y;
        }

        point3d& operator+=(const point3d& p2);

        friend point3d operator-(const point3d& p1, const point3d& p2)
        {
            point3d temp(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
            return temp;
        }

        point3d& operator -=(const point3d& p1)
        {
            x() -= p1.x();
            y() -= p1.y();
            z() -= p1.z();
            return *this;
        }

        point3d operator *(const D mult)
        {
            point3d temp(*this);
            std::transform(temp.coord_.begin(), temp.coord_.end(), temp.coord_.begin(), [&](const D c)
            {
                return c * mult;
            });
            return temp;
        }

        point3d& operator*=(const D mult)
        {
            std::transform(coord_.begin(), coord_.end(), coord_.begin(), [&](const D c)
            {
                return c * mult;
            });
            return *this;
        }

        friend std::ostream& operator <<(std::ostream& out, const point3d& p);

        static point3d angle(const point3d& p1, const point3d& p2)
        {
            point3d point(p2);
            point -= p1;
            D ax(std::atan2(std::sqrt(point.y() * point.y() + point.z() * point.z()), point.x()));
            D ay(std::atan2(std::sqrt(point.z() * point.z() + point.x() * point.x()), point.y()));
            D az(std::atan2(std::sqrt(point.x() * point.x() + point.y() * point.y()), point.z()));
            point.x() = std::cos(ax);
            point.y() = std::cos(ay);
            point.z() = std::cos(az);
            return point;
        }
    };

    D point3d::distance(const point3d& p1, const point3d& p2)
    {
        return std::sqrt(distance2(p1, p2));
    }

    D point3d::distance2(const point3d& p1, const point3d& p2)
    {
        std::array<D, 3> tmp;
        std::transform(p1.coord_.begin(), p1.coord_.end(), p2.coord_.begin(), tmp.begin(), [](const D x1, const D x2)
        {
            return (x1 - x2) * (x1 - x2);
        });
        return std::accumulate(tmp.begin(), tmp.end(), 0.0);
    }

    class sys_object: public point3d {
    public:
        D mass_;
        point3d velocity_;
        point3d acceleraion_;
        std::vector<std::shared_ptr<sys_object>> satelites_;

    public:
        sys_object() :
                point3d(), mass_(), velocity_(), acceleraion_(), satelites_()
        {
        }
        sys_object(D mass) :
                point3d(), mass_(mass), velocity_(), acceleraion_(), satelites_()
        {
        }
    };

    class star: public sys_object {
    public:
        star() :
                sys_object()
        {
        }
        star(D mass) :
                sys_object(mass)
        {
        }
    };

    class planet: public sys_object {
        const std::string name_;
        std::shared_ptr<planet> master_;
    public:
        planet() :
                sys_object(), name_("Undefined"), master_()
        {
        }
        planet(const std::string& name, D x0, D y0, D z0) :
                sys_object(), name_(name), master_()
        {
            x() = x0;
            y() = y0;
            z() = z0;
        }

        std::string name() const
        {
            return name_;
        }

        friend std::ostream& operator <<(std::ostream& out, const planet& plan);

        point3d position() const
        {
            return *this;
        }

        static std::shared_ptr<planet> declare_planet(std::shared_ptr<planet> master, const std::string& name, D distance,
                D mass, D angle, D velocity);

        static std::shared_ptr<planet> declare_planet(const std::string& name, D distance, D mass, D angle,
                D velocity);
    };

    class solar_system {
        star sun_;
        std::vector<std::shared_ptr<planet>> planets_;

    public:
        constexpr static D AU = 149497870700.0;

        solar_system() :
                sun_(1.98855e30), planets_()
        {
        }

        friend std::ostream& operator <<(std::ostream& out, const solar_system& sol);

        void define_planet(std::shared_ptr<planet>&& planet)
        {
            planets_.push_back(planet);
        }

        void define_planet(std::shared_ptr<planet>& planet)
        {
            planets_.push_back(planet);
        }

        void step(D dT);
        static D calculateGravity(const sys_object& p1, const sys_object& p2);

        const star& sun() const
        {
            return sun_;
        }
        const std::vector<std::shared_ptr<planet>>& planets() const
        {
            return planets_;
        }

    private:
        void calculateAcceleration(D dT);
        void calculateVelocity(D dT);
        void move(D dT);

        point3d calculateForce(const sys_object& p1, const sys_object& p2);
    };

    class planet_track;
    class drawing_engine {
    private:
        geometry::matrix44ld Mproj_;
        geometry::matrix44ld worldToCamera_;

    public:
        constexpr static uint32_t imageWidth_ = 1024;
        constexpr static uint32_t imageHeight_ = 1024;

    public:
        drawing_engine()
        {
            //        worldToCamera[3][0] = 3.5;
            //        worldToCamera[3][1] = 3.5;
            worldToCamera_[3][2] = 3.5;
            D angleOfView = 50.0;
            D near = 0.1;
            D far = 7.0;
            D imageAspectRatio = imageWidth_ / (D) imageHeight_;
            D b, t, l, r;
            gluPerspective(angleOfView, imageAspectRatio, near, far, b, t, l, r);
            glFrustum(b, t, l, r, near, far, Mproj_);
        }

        void plot(std::ostream& out, const std::vector<planet_track>& history);

    private:
        void computePixelCoordinates(const geometry::vector3ld pWorld, geometry::vector2i &pRaster,
                const geometry::matrix44ld &worldToCamera, const D &canvasWidth, const D &canvasHeight,
                const uint32_t &imageWidth, const uint32_t &imageHeight)
        {
            geometry::vector3ld pCamera(pWorld * worldToCamera);
            geometry::vector2ld pScreen(pCamera.x / -pCamera.z, pCamera.y / -pCamera.z);
            geometry::vector2ld pNDC((pScreen.x + canvasWidth * 0.5) / canvasWidth, (pScreen.y + canvasHeight * 0.5) / canvasHeight);
            pRaster.x = (int) (pNDC.x * imageWidth);
            pRaster.y = (int) ((1 - pNDC.y) * imageHeight);
        }

        void gluPerspective(const D &angleOfView, const D &imageAspectRatio, const D &n, const D &f,
                D &b, D &t, D &l, D &r)
        {
            D scale = tan(angleOfView * 0.5 * M_PI / 180) * n;
            r = imageAspectRatio * scale, l = -r;
            t = scale, b = -t;
        }

        void glFrustum(const D &b, const D &t, const D &l, const D &r, const D &n,
                const D &f, geometry::matrix44ld &M)
        {
            // set OpenGL perspective projection matrix
            M[0][0] = 2 * n / (r - l);
            M[0][1] = 0;
            M[0][2] = 0;
            M[0][3] = 0;

            M[1][0] = 0;
            M[1][1] = 2 * n / (t - b);
            M[1][2] = 0;
            M[1][3] = 0;

            M[2][0] = (r + l) / (r - l);
            M[2][1] = (t + b) / (t - b);
            M[2][2] = -(f + n) / (f - n);
            M[2][3] = -1;

            M[3][0] = 0;
            M[3][1] = 0;
            M[3][2] = -2 * f * n / (f - n);
            M[3][3] = 0;
        }

        geometry::vector2i translatePoint(const geometry::vector3ld& phys);
    };

    class planet_track {
    public:
        std::string name_;
        std::deque<geometry::vector3ld> track_;
    };

    class presentation {
        std::vector<planet_track> history_;
        drawing_engine de_;

        void calculateDensity(const image::ppm& out, std::vector<std::vector<D> >& field, const solar_system& sys,
                D& maxPow, D& minPow);

    public:
        presentation() :
                history_(), de_()
        {
        }
        void start_recording(const solar_system& sys);
        void record_track(const solar_system& sys);
        void generate_view(const solar_system& sys, const D sim_time);
    };

    class simulator {
        std::shared_ptr<solar_system> system_;
        std::shared_ptr<presentation> presentation_;
        D time_;
        constexpr static D MINUTE = 1.0 * 60.0;
        constexpr static D HOUR = MINUTE * 60.0;
        constexpr static D DAY = HOUR * 24.0;

    public:
        simulator() :
                system_(std::make_shared<solar_system>()), presentation_(std::make_shared<presentation>()), time_(0)
        {
            system_->define_planet(planet::declare_planet("Mercury", 0.466697, 3.3011e23, 3.38, 47362.0));
            system_->define_planet(planet::declare_planet("Venus", 0.7, 4.8675e24, 3.86, 35020.0));
            auto earh = planet::declare_planet("Earh", 1.0, 5.97237e24, 7.155, 29780.0);
            system_->define_planet(earh);
            system_->define_planet(planet::declare_planet(earh, "Moon", 405400.0, 7.342e22, 5.145, 1022.0));
            system_->define_planet(planet::declare_planet("Mars", 1.666, 64171e23, 5.65, 24077.0));
        }

        void step();

        friend std::ostream& operator <<(std::ostream& out, const simulator& sim);

    };

    std::shared_ptr<planet> planet::declare_planet(const std::string& name, D distance, D mass, D angle,
            D velocity)
    {
        angle = angle * M_PIl / 180.0;
        point3d position(distance * solar_system::AU, 0.0, 0.0);
        position.rotate_y(-angle);
        auto plan = std::make_shared<planet>(name, position.x(), position.y(), position.z());
        plan->velocity_.y() = velocity;
        plan->mass_ = mass;
        return plan;
    }

    std::shared_ptr<planet> planet::declare_planet(std::shared_ptr<planet> master, const std::string& name, D distance,
            D mass, D angle, D velocity)
    {
        angle = angle * M_PIl / 180.0;
        point3d position(distance * 1000.0, 0.0, 0.0);
        position.rotate_y(-angle);
        position += master->position();
        auto plan = std::make_shared<planet>(name, position.x(), position.y(), position.z());
        master->satelites_.push_back(plan);
        plan->velocity_.y() = velocity + master->velocity_.y();
        plan->mass_ = mass;
        return plan;
    }

    point3d& point3d::operator +=(const point3d& p2)
    {
        coord_[0] += p2.coord_[0];
        coord_[1] += p2.coord_[1];
        coord_[2] += p2.coord_[2];
        return *this;
    }

    void simulator::step()
    {
        if (time_ == 0)
        {
            presentation_->start_recording(*system_);
        }
        else if (((long long) time_ % 100) == 0)
        {
            presentation_->generate_view(*system_, time_);
        }
        time_++;
        system_->step(DAY);
        presentation_->record_track(*system_);
    }

    void solar_system::step(D dT)
    {
        calculateAcceleration(dT);
        calculateVelocity(dT);
        move(dT);
    }

    void solar_system::calculateVelocity(D dT)
    {
        std::for_each(planets_.begin(), planets_.end(), [&](const std::shared_ptr<planet>& plan)
        {
            plan->velocity_ += plan->acceleraion_ * dT;
        });
    }

    void solar_system::move(D dT)
    {
        std::for_each(planets_.begin(), planets_.end(), [&](const std::shared_ptr<planet>& plan)
        {
            *plan += plan->velocity_ * dT;
        });
    }

    void solar_system::calculateAcceleration(D dT)
    {
        std::for_each(planets_.begin(), planets_.end(), [&](const std::shared_ptr<planet>& plan)
        {
            point3d force(calculateForce(sun_, *plan));
            std::for_each(planets_.begin(), planets_.end(), [&](const std::shared_ptr<planet>& plan2)
                    {
                        if (plan->distance(*plan2) > 0.0)
                        {
                            force += calculateForce(*plan2, *plan);
                        }
                    });
            plan->acceleraion_ = force * (-1.0 / plan->mass_);
        });
    }

    D solar_system::calculateGravity(const sys_object& p1, const sys_object& p2)
    {
        return 6.67408e-11 * p1.mass_ * p2.mass_ / point3d::distance2(p1, p2);
    }

    point3d solar_system::calculateForce(const sys_object& p1, const sys_object& p2)
    {
        D temp(calculateGravity(p1, p2));
        point3d result(point3d::angle(p1, p2));
        result *= temp;
        return result;
    }

    void presentation::start_recording(const solar_system& sys)
    {
        history_.reserve(sys.planets().size());
        std::transform(sys.planets().begin(), sys.planets().end(), std::back_inserter(history_), [](const std::shared_ptr<planet>& plan)
        {
            planet_track track;
            track.name_ = plan->name();
            track.track_.push_back(
                    {   plan->x(), plan->y(), plan->z()});
            return track;
        });
    }

    void presentation::record_track(const solar_system& sys)
    {
        auto hIt = history_.begin();
        std::for_each(sys.planets().begin(), sys.planets().end(), [&](const std::shared_ptr<planet>& plan)
        {
            if (hIt->track_.size() > 100)
            {
                hIt->track_.pop_back();
            }
            hIt->track_.push_front(
                    {   plan->x(), plan->y(), plan->z()});
            ++hIt;
        });
    }

    void presentation::calculateDensity(const image::ppm& out, std::vector<std::vector<D> >& field, const solar_system& sys,
            D& maxPow, D& minPow)
    {
        D y = 2 * solar_system::AU;
        D dx = 4 * solar_system::AU / out.width;
        D dy = 4 * solar_system::AU / out.height;
        sys_object point(1.0);
        for (uint32_t iy = 0; iy < out.height; iy++)
        {
            D x = -2 * solar_system::AU;
            field[iy].resize(out.width);
            for (uint32_t ix = 0; ix < out.width; ix++)
            {
                point.x() = x;
                point.y() = y;
                if (point3d::distance2(point, sys.sun()) != 0.0)
                {
                    D power(sys.calculateGravity(point, sys.sun()));
                    std::for_each(sys.planets().begin(), sys.planets().end(), [&](const std::shared_ptr<planet>& plan)
                    {
                        if (point3d::distance2(point, *plan) != 0.0)
                        {
                            power += sys.calculateGravity(point, *plan);
                        }
                    });
                    field[iy][ix] = power;
                    maxPow = std::max(maxPow, power);
                    minPow = std::min(minPow, power);
                }
                x += dx;
            }
            y -= dy;
        }
    }

    geometry::vector2i drawing_engine::translatePoint(const geometry::vector3ld& phys)
    {
        geometry::vector3ld camera((phys / solar_system::AU) * worldToCamera_);
        geometry::vector3ld projected(camera * Mproj_);
        geometry::vector2i point(std::min(imageWidth_ - 1, (uint32_t) (((projected.x + 1) * 0.5 * imageWidth_))),
                std::min(imageHeight_ - 1, (uint32_t) (((1 - (projected.y + 1) * 0.5) * imageHeight_))));
        return point;
    }

    void drawing_engine::plot(std::ostream& out, const std::vector<planet_track>& history)
    {
        out << "<svg version=\"1.1\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns=\"http://www.w3.org/2000/svg\" height=\""
                << imageHeight_ << "\" width=\"" << imageWidth_ << "\">" << std::endl;
        out << "<rect x=\"0\" y=\"0\" height=\"" << imageHeight_ << "\" width=\"" << imageWidth_ << "\" />" << std::endl;
        std::for_each(history.rbegin(), history.rend(), [&](const planet_track& plan)
        {
            geometry::vector2i lStart(translatePoint(plan.track_.front()));
            out << "<circle cx=\"" << lStart.x << "\" cy=\"" << lStart.y << "\" r=\"3\" style=\"fill:blue; stroke:none\" />" << std::endl;
            out << "<path d=\"M " << lStart.x << " " << lStart.y << " L";
            std::for_each(plan.track_.begin(), plan.track_.end(), [&](const geometry::vector3ld& point)
            {
                geometry::vector2i lEnd (translatePoint(point));
                if (lStart.x != lEnd.x || lStart.y != lEnd.y)
                {
                    out << " " << lEnd.x << " " << lEnd.y;
                    lStart = lEnd;
                }
            });
            out << "\" style=\"stroke:green; stroke-width:1\" />" << std::endl;
        });
        geometry::vector2i lsun(translatePoint( { 0, 0, 0 }));
        out << "<circle cx=\"" << lsun.x << "\" cy=\"" << lsun.y << "\" r=\"10\" style=\"fill:yellow; stroke:none\" />" << std::endl;
        out << "</svg>" << std::endl;

    }

    void presentation::generate_view(const solar_system& sys, const D sim_time)
    {
        std::stringstream fname;
        fname << "solar_" << sim_time << ".svg";
        std::ofstream out(fname.str());
        de_.plot(out, history_);
    }

}  // namespace gravity

std::ostream& gravity::operator <<(std::ostream& out, const gravity::point3d& p)
{
    out << "[" << p.coord_[0] << ", " << p.coord_[1] << ", " << p.coord_[2] << "]";
    return out;
}

std::ostream& gravity::operator <<(std::ostream& out, const gravity::planet& plan)
{
    out << plan.name_ << std::endl;
    out << "\t coordinates " << *((gravity::point3d*) &plan) << std::endl;
    out << "\t    velocity " << plan.velocity_ << std::endl;
    out << "\tacceleration " << plan.acceleraion_ << std::endl;
    return out;
}

std::ostream& gravity::operator <<(std::ostream& out, const gravity::solar_system& sol)
{
    std::for_each(sol.planets_.begin(), sol.planets_.end(), [&](const std::shared_ptr<gravity::planet>& plan)
    {
        out << *plan << std::endl;
    });
    return out;
}

std::ostream& gravity::operator <<(std::ostream& out, const gravity::simulator& sim)
{
    out << "Time: " << sim.time_ << std::endl;
    out << *sim.system_ << std::endl;
    return out;
}

void grav()
{
    gravity::simulator sim;
    std::cout << sim << std::endl;
    while (true)
    {
        sim.step();
        std::cout << sim << std::endl;
    }
}
