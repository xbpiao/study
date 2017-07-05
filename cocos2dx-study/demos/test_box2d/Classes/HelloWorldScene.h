#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "cocos2d.h"
#include "Box2D/Box2D.h"
#include "Mybox2dWorld.h"

USING_NS_CC;

/* 尽量不使用过期(deprecated)的方法 */

class HelloWorld : public cocos2d::Layer
{
public:
    virtual ~HelloWorld();
    virtual bool init() override;

    static cocos2d::Scene* scene();

    // a selector callback
    void menuCloseCallback(Ref* sender);

    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);

protected:

private:
    Mybox2dWorld* _myWorld = nullptr;
};

#endif // __HELLOWORLD_SCENE_H__
