#include "bullet.hpp"



namespace rmMultistage {
    using rm_common::Point2D;

    Bullet::Bullet(std::string shooter,double x,double y,double yaw,double speed):
        shooter_(shooter),
        speed_(speed),
        yaw_(yaw),
        origin_(x,y),
        position_now_(x,y),
        position_last_(x,y),
        coveredDistance_(0)
    {

    }

    void Bullet::Move(int frequency)
    {
        position_last_ = position_now_;
        double dx = std::cos(yaw_) * speed_ / frequency;
        double dy = std::sin(yaw_) * speed_ / frequency;
        coveredDistance_ += speed_ / frequency;
        position_now_ += Point2D (dx,dy);
    }


    rm_decision_interfaces::msg::BulletMove Bullet::GetBulletMove()
    {
        rm_decision_interfaces::msg::BulletMove bulletMove;
        bulletMove.owner = shooter_;
        bulletMove.x = position_now_.X();
        bulletMove.y = position_now_.Y();
        bulletMove.x_last = position_last_.X();
        bulletMove.y_last = position_last_.Y();
        bulletMove.yaw = yaw_;
        bulletMove.speed = speed_;
        bulletMove.covered_distance = coveredDistance_;
        return bulletMove;

    }

    bool Bullet::ReachBoundary(int xmax, int ymax)
    {
        return (position_now_.X() > xmax || position_now_.Y() > ymax||position_now_.X() < 0 || position_now_.Y() < 0);
    }


}








