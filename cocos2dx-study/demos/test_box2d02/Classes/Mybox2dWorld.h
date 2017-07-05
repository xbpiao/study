//
//  Mybox2dWorld.h
//  test_box2d
//
//  Created by xiebin on 2017/3/18.
//
//

#ifndef Mybox2dWorld_h
#define Mybox2dWorld_h

#include "cocos2d.h"
#include "Box2D/Box2D.h"
#include "extensions/cocos-ext.h"
#include "GLES-Render.h"

USING_NS_CC;
USING_NS_CC_EXT;

/* 监听 */

class Myb2ContactListener : public b2ContactListener
{
    public:
    
    virtual void BeginContact(b2Contact* contact) ;
    
    /// Called when two fixtures cease to touch.
    virtual void EndContact(b2Contact* contact);
    
    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. disable contact).
    /// A copy of the old manifold is provided so that you can detect changes.
    /// Note: this is called only for awake bodies.
    /// Note: this is called even when the number of contact points is zero.
    /// Note: this is not called for sensors.
    /// Note: if you set the number of contact points to zero, you will not
    /// get an EndContact callback. However, you may get a BeginContact callback
    /// the next step.
    virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
    
    /// This lets you inspect a contact after the solver is finished. This is useful
    /// for inspecting impulses.
    /// Note: the contact manifold does not include time of impact impulses, which can be
    /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    /// in a separate data structure.
    /// Note: this is only called for contacts that are touching, solid, and awake.
    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
    
private:
    
    void destorySprite(PhysicsSprite* sprite);
    
};

/* 简单实现一个box2d的环境 */
class Mybox2dWorld : public cocos2d::Node
{
public:
    virtual ~Mybox2dWorld();
    virtual bool init() override;
    
    // implement the "static create()" method manually
    CREATE_FUNC(Mybox2dWorld);
    
    virtual void update(float dt) override;
    
    virtual void draw(Renderer *renderer, const Mat4& transform, uint32_t flags) override;
protected:
    void initPhysics();
private:
    void addNewSpriteAtPosition(cocos2d::Vec2 p);
    void onTouchesBegan(const std::vector<Touch*>& touches, Event *unused_event);
    void onTouchesMoved(const std::vector<Touch*>& touches, Event *unused_event);
    void onTouchesEnded(const std::vector<Touch*>& touches, Event *unused_event);
    
    void onDraw(const cocos2d::Mat4& transform, uint32_t flags);
    cocos2d::Mat4               _modelViewMV;
    b2World*                    _world = nullptr;
    Texture2D*                  _spriteTexture = nullptr;
    Texture2D*                  _spriteBombTexture = nullptr; // 炸弹的图片
    Node*                       _blockParent = nullptr;
    GLESDebugDraw               _debugDraw;
    uint32                      _debugDrawflags;
    cocos2d::CustomCommand      _customCmd;
    Vec2                        _beganPos;
    Vec2                        _movedPos;
    Sprite*                     _beganSprite;
    Myb2ContactListener         _myb2ContactListener;
    int                         _randomIdx;
    int                         _randomIdy;
};


#endif /* Mybox2dWorld_h */
