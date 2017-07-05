//
//  Mybox2dWorld.cpp
//  test_box2d
//
//  Created by xiebin on 2017/3/18.
//
//

#include "Mybox2dWorld.h"
#include "VisibleRect.h"

// 关于PTM_RATIO
// http://blog.csdn.net/plus5/article/details/16371313
// This is defining a ratio of pixels to “meters”. When you specify where bodies are in Cocos2D, you give it a set of units.
// Although you may consider using pixels, that would be a mistake. According to the Box2D manual,
// Box2D has been optimized to deal with units as small as 0.1 and as big as 10.
// So as far as length goes people generally tend to treat it as “meters” so 0.1 would be about teacup size and 10 would be about box size.
// So we don’t want to pass pixels in, because even small objects would be 60×60 pixels,
// way bigger than the values Box2D has been optimized for. So we need to have a way to convert pixels to “meters”,
// hence we can just define a ratio like the above.
// So if we had a 64 pixel object, we could divide it by PTM_RATIO to get 2 “meters” that Box2D can deal with for physics simulation purposes.

// PTM_RATIO 32是定义宏PTM_RATIO，PTM_RATIO是屏幕上多少Point为1米，32表示屏幕上32Point表示1米，
// 在Box2D中单位使用MKS公制系统，即：长度单位采用米，质量单位采用千克，时间单位采用秒。
#define PTM_RATIO 32

void Myb2ContactListener::BeginContact(b2Contact* contact)
{
    b2Contact* next = contact;
    int n = 0;
    while (next) {
        n++;
        
        b2Body* bodyA = next->GetFixtureA()->GetBody();
        b2Body* bodyB = next->GetFixtureB()->GetBody();
        if (bodyA && bodyB)
        {
            PhysicsSprite* pSpriteA = dynamic_cast<PhysicsSprite*>((Node*)bodyA->GetUserData());
            PhysicsSprite* pSpriteB = dynamic_cast<PhysicsSprite*>((Node*)bodyB->GetUserData());
            if (pSpriteA && pSpriteB)
            {
                pSpriteA->setColor(Color3B::RED);
                pSpriteB->setColor(Color3B::RED);
                
                bodyA->SetUserData(nullptr);
                bodyB->SetUserData(nullptr);
                
                pSpriteA->stopAllActions();
                pSpriteA->runAction(Sequence::create(FadeOut::create(1.5f),
                                                     CallFunc::create(std::bind(&Myb2ContactListener::destorySprite, this, pSpriteA)), nullptr));
                pSpriteB->stopAllActions();
                pSpriteB->runAction(Sequence::create(FadeOut::create(1.5f),
                                                     CallFunc::create(std::bind(&Myb2ContactListener::destorySprite, this, pSpriteB)), nullptr));
                
                // 不能这样删除
                // bodyA->GetWorld()->DestroyBody(bodyA);
                // bodyB->GetWorld()->DestroyBody(bodyB);
            }
            else
            {
                /*
                if (pSpriteA)
                {
                    if (pSpriteA->getColor() == Color3B::RED)
                    {
                        pSpriteA->setColor(Color3B::WHITE);
                    }
                }// if
                
                if (pSpriteB)
                {
                    if (pSpriteB->getColor() == Color3B::RED)
                    {
                        pSpriteB->setColor(Color3B::WHITE);
                    }
                }// if
                 */
            }// if
        }// if
        
        next = next->GetNext();
    }// while
    
    CCLOG("BeginContact n=%d", n);
}

void Myb2ContactListener::EndContact(b2Contact* contact)
{
    b2Contact* next = contact;
    int n = 0;
    while (next) {
        n++;
        
        b2Body* body = next->GetFixtureA()->GetBody();
        if (body)
        {
            PhysicsSprite* pSprite = dynamic_cast<PhysicsSprite*>((Node*)body->GetUserData());
            if (pSprite)
            {
                if (pSprite->getColor() == Color3B::RED)
                {
                    pSprite->setColor(Color3B::WHITE);
                }
            }// if
        }// if
        
        next = next->GetNext();
    }// while
    
    CCLOG("EndContact n=%d", n);
    
}

void Myb2ContactListener::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
    /*
    b2Contact* next = contact;
    int n = 0;
    while (next) {
        n++;
        
        b2Body* body = next->GetFixtureA()->GetBody();
        if (body)
        {
            PhysicsSprite* pSprite = dynamic_cast<PhysicsSprite*>((Node*)body->GetUserData());
            if (pSprite)
            {

            }// if
        }// if
        
        next = next->GetNext();
    }// while
    
    CCLOG("PreSolve n=%d", n);
     */
}


void Myb2ContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
    /*
    b2Contact* next = contact;
    int n = 0;
    while (next) {
        n++;
        
        b2Body* body = next->GetFixtureA()->GetBody();
        if (body)
        {
            PhysicsSprite* pSprite = dynamic_cast<PhysicsSprite*>((Node*)body->GetUserData());
            if (pSprite)
            {
                
            }// if
        }// if
        
        next = next->GetNext();
    }// while
    
    CCLOG("PostSolve n=%d", n);
     */
}

void Myb2ContactListener::destorySprite(PhysicsSprite* sprite)
{
    if (sprite)
    {
        b2Body* body = sprite->getB2Body();
        sprite->setB2Body(nullptr);
        if (body)
        {
            body->GetWorld()->DestroyBody(body);
        }// if
        sprite->removeFromParent();
        // delete sprite;
    }
}

// ======================================================================================

Mybox2dWorld::~Mybox2dWorld()
{
    CC_SAFE_DELETE(_world);
}

bool Mybox2dWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Node::init() )
    {
        return false;
    }
    
    /* 监听触摸事件 */
    auto dispatcher = Director::getInstance()->getEventDispatcher();
    auto touchListener = EventListenerTouchAllAtOnce::create();
    touchListener->onTouchesBegan = CC_CALLBACK_2(Mybox2dWorld::onTouchesBegan, this);
    touchListener->onTouchesMoved = CC_CALLBACK_2(Mybox2dWorld::onTouchesMoved, this);
    touchListener->onTouchesEnded = CC_CALLBACK_2(Mybox2dWorld::onTouchesEnded, this);
    dispatcher->addEventListenerWithSceneGraphPriority(touchListener, this);
    
    _spriteTexture = Director::getInstance()->getTextureCache()->addImage("images/blocks.png");
    CCLOG("_spriteTexture->getContentSize()=(%.1f , %.1f)",
          _spriteTexture->getContentSize().width,
          _spriteTexture->getContentSize().height);
    CCLOG("_spriteTexture->getContentSizeInPixels()=(%.1f , %.1f)",
          _spriteTexture->getContentSizeInPixels().width,
          _spriteTexture->getContentSizeInPixels().height);
    
    _spriteBombTexture = Director::getInstance()->getTextureCache()->addImage("images/77.png");
    
    _beganSprite = Sprite::createWithTexture(_spriteBombTexture);
    _beganSprite->setPosition(Vec2(200, 200));
    addChild(_beganSprite);
    _beganSprite->setVisible(false);
    
    
    // 所有的动态创建节点存放处
    _blockParent = Node::create();
    addChild(_blockParent, 0, 1);
    
    initPhysics();
    scheduleUpdate();
     
    return true;
}

void Mybox2dWorld::initPhysics()
{
    b2Vec2 gravity;
    // 设置重力加速度（发现设置重力 gravity 不受分辨率影响）
    gravity.Set(0.0f, -10.0f);
    _world = new b2World(gravity);
    
    // Do we want to let bodies sleep?
    _world->SetAllowSleeping(true);
    
    _world->SetContinuousPhysics(true);
    
    // _world->SetContactListener(&_myb2ContactListener);
    
    _debugDrawflags = 0;
    
    _debugDrawflags += b2Draw::e_shapeBit;
    _debugDrawflags += b2Draw::e_jointBit;
    _debugDrawflags += b2Draw::e_aabbBit; // 包围盒
    _debugDrawflags += b2Draw::e_pairBit;
     
    _debugDrawflags += b2Draw::e_centerOfMassBit; // 显示重心位置
    
    _debugDraw.SetFlags(_debugDrawflags);
    _world->SetDebugDraw(&_debugDraw);
    
    // Define the ground body.
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0, 0); // bottom-left corner
    
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    b2Body* groundBody = _world->CreateBody(&groundBodyDef);
    
    // Define the ground box shape.
    b2EdgeShape groundBox;
    
    // bottom
    groundBox.Set(b2Vec2(VisibleRect::leftBottom().x/PTM_RATIO,VisibleRect::leftBottom().y/PTM_RATIO), b2Vec2(VisibleRect::rightBottom().x/PTM_RATIO,VisibleRect::rightBottom().y/PTM_RATIO));
    groundBody->CreateFixture(&groundBox,0);
    
    // top
    groundBox.Set(b2Vec2(VisibleRect::leftTop().x/PTM_RATIO,VisibleRect::leftTop().y/PTM_RATIO), b2Vec2(VisibleRect::rightTop().x/PTM_RATIO,VisibleRect::rightTop().y/PTM_RATIO));
    groundBody->CreateFixture(&groundBox,0);
    
    // left
    groundBox.Set(b2Vec2(VisibleRect::leftTop().x/PTM_RATIO,VisibleRect::leftTop().y/PTM_RATIO), b2Vec2(VisibleRect::leftBottom().x/PTM_RATIO,VisibleRect::leftBottom().y/PTM_RATIO));
    groundBody->CreateFixture(&groundBox,0);
    
    // right
    groundBox.Set(b2Vec2(VisibleRect::rightBottom().x/PTM_RATIO,VisibleRect::rightBottom().y/PTM_RATIO), b2Vec2(VisibleRect::rightTop().x/PTM_RATIO,VisibleRect::rightTop().y/PTM_RATIO));
    groundBody->CreateFixture(&groundBox,0);
}

void Mybox2dWorld::update(float dt)
{
    //It is recommended that a fixed time step is used with Box2D for stability
    //of the simulation, however, we are using a variable time step here.
    //You need to make an informed choice, the following URL is useful
    //http://gafferongames.com/game-physics/fix-your-timestep/
    
    int velocityIterations = 8;
    int positionIterations = 1;
    
    static float num_dt = 0.0f;
    static float box2d_dt = (1.0f/60.0f);
    num_dt += dt;
    
    int k = num_dt / box2d_dt;
    
    if (k >= 1)
    {// 保证box2d总是有60帧/秒的速率
        for (int i = 0; i < k; i++)
        {
            _world->Step(box2d_dt, velocityIterations, positionIterations);
        }// for i
        num_dt = num_dt - k * box2d_dt;
    }// if
    
    // Instruct the world to perform a single step of simulation. It is
    // generally best to keep the time step and iterations fixed.
    
    // _world->Step(dt, velocityIterations, positionIterations);
}

void Mybox2dWorld::addNewSpriteAtPosition(cocos2d::Vec2 p)
{
    CCLOG("Add sprite %0.2f x %02.f",p.x,p.y);
    
    // Define the dynamic body.
    //Set up a 1m squared box in the physics world
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    // 注意传进来的p已经是point的坐标不是像素(pixels)的
    bodyDef.position.Set(p.x/PTM_RATIO, p.y/PTM_RATIO);
    
    Vec2 n = _movedPos - _beganPos;
    float a = M_PI - n.getAngle(Vec2::UNIT_X);
    // 要转换为弧度
    bodyDef.angle = a;
    CCLOG("Mybox2dWorld::addNewSpriteAtPosition angle=%.2f", a);
    
    // http://ohcoder.com/blog/2012/11/30/anatomy-of-a-collision/
    // Bullte物体会花费更多CPU时间来计算碰撞点，而且对于很多应用来说这是没有必要的。
    // 作为默认设置，你需要知道有时候会发生碰撞完全丢失的情况－比如本例中，如果三角形移动过快，完全有肯能跳过四方盒子的右上角！
    // 如果一个物体需要运动的足够快而且又不能丢失任何碰撞，例如呃～…子弹！:)那么你需要把物体设置成bullet物体。
    // 接下来，我们还是延续之前的非bullet物体属性进行讨论。
    // 如果SetBullet(false)则是相交后才检测碰撞，SetBullet(true)则是相交前检测
    bodyDef.bullet = true;
    
    // fixedRotation = true;则发射角度就固定了，掉下来碰撞也没有角度变化
    // bodyDef.fixedRotation = true;
    
    b2Body *body = _world->CreateBody(&bodyDef);
    
    float c = 0.0f;
    
    // 弹头，密度大一些
    {
        // "images/77.png" 宽62 高33
        b2PolygonShape dynamicBombHead;
        Size bombHeadSize = CC_SIZE_PIXELS_TO_POINTS(Size(12.0f, 19.0f) / PTM_RATIO) / 2.0f;
        Vec2 bombHeadBoxPos = CC_POINT_PIXELS_TO_POINTS(Vec2(12.0f, 0.0f) / PTM_RATIO);
        dynamicBombHead.SetAsBox(bombHeadSize.width, bombHeadSize.height, b2Vec2(bombHeadBoxPos.x, bombHeadBoxPos.y), 0.0f);
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDefBombHead;
        fixtureDefBombHead.shape = &dynamicBombHead;
        fixtureDefBombHead.density = 2.5f;  // 密度
        fixtureDefBombHead.friction = 0.3f; // 摩擦
        body->CreateFixture(&fixtureDefBombHead);
        
        c += bombHeadSize.width * bombHeadSize.height * fixtureDefBombHead.density;
    }
    
    {// 弹身，密度正常
        // "images/77.png" 宽62 高33
        b2PolygonShape dynamicBombBody;
        Size bombBodySize = CC_SIZE_PIXELS_TO_POINTS(Size(24.0f, 15.0f) / PTM_RATIO) / 2.0f;
        Vec2 bombBodyPos = CC_POINT_PIXELS_TO_POINTS(Vec2(-6.0f, 0.0f) / PTM_RATIO);
        dynamicBombBody.SetAsBox(bombBodySize.width, bombBodySize.height, b2Vec2(bombBodyPos.x, bombBodyPos.y), 0.0f);
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDefBombBody;
        fixtureDefBombBody.shape = &dynamicBombBody;
        fixtureDefBombBody.density = 0.2f;  // 密度
        fixtureDefBombBody.friction = 0.3f; // 摩擦
        body->CreateFixture(&fixtureDefBombBody);
        c += bombBodySize.width * bombBodySize.height * fixtureDefBombBody.density;
    }
    
    b2Vec2 tailAppleforcePos = bodyDef.position;
    
    {// 弹尾，密度正常
        // "images/77.png" 宽62 高33
        b2PolygonShape dynamicBombTail;
        Size bombTailSize = CC_SIZE_PIXELS_TO_POINTS(Size(9.0f, 20.0f) / PTM_RATIO) / 2.0f;
        Vec2 bombTailPos = CC_POINT_PIXELS_TO_POINTS(Vec2(-22.0f, 0.0f) / PTM_RATIO);
        dynamicBombTail.SetAsBox(bombTailSize.width, bombTailSize.height, b2Vec2(bombTailPos.x, bombTailPos.y), 0.0f);
        // Define the dynamic body fixture.
        b2FixtureDef fixtureDefBombTail;
        fixtureDefBombTail.shape = &dynamicBombTail;
        fixtureDefBombTail.density = 0.2f;  // 密度
        fixtureDefBombTail.friction = 0.3f; // 摩擦
        body->CreateFixture(&fixtureDefBombTail);
        c += bombTailSize.width * bombTailSize.height * fixtureDefBombTail.density;
        tailAppleforcePos = tailAppleforcePos + b2Vec2(bombTailPos.x, bombTailPos.y);
    }
    
    // 第一个参数施加的力，单位牛顿
    // 第二个参数施加力的坐标
    // body->ApplyForce(b2Vec2(60, 120), bodyDef.position, false);
    // 这个力的单位也会随分辨率变化，我去，这里也要计算出一个准确的值，让不同分辨率的效果一致，采用缩放也是种方案
    float forceX, forceY;
    forceX = 318 * body->GetMass();
    forceY = 318 * body->GetMass();
    
    /*
    b2MassData mdata;
    Vec2 bombMassPos = CC_POINT_PIXELS_TO_POINTS(Vec2(0.0f, 0.0f) / PTM_RATIO);
    mdata.center = b2Vec2(bombMassPos.x, bombMassPos.y);
    body->SetMassData(&mdata);
     */
    
    // 计算力的方向
    
    Vec2 v = _movedPos - _beganPos;
    v.normalize();
    forceX = -forceX * v.x;
    forceY = -forceY * v.y;
    
    
    CCLOG("forceX=%.3f forceY=%.3f v_dir(%.2f, %.2f)", forceX, forceY, v.x, v.y);
    
    
    // body->SetAngularVelocity(10);
    body->ApplyForce(b2Vec2(forceX, forceY), b2Vec2(bodyDef.position.x, bodyDef.position.y), true);
    
    // body->ApplyLinearImpulse(<#const b2Vec2 &impulse#>, <#const b2Vec2 &point#>, <#bool wake#>);
    // body->SetLinearDamping(1.2f);
    
   
    /* 注意:createWithTexture传入的RECT居然不是以像素为单位的，见 */
    auto sprite = PhysicsSprite::createWithTexture(_spriteBombTexture);
    _blockParent->addChild(sprite);
    sprite->setB2Body(body);
    sprite->setPTMRatio(PTM_RATIO);
    sprite->setPosition(cocos2d::Vec2(p.x, p.y));
    sprite->setOpacity(80);
    
    // 关联显示的sprite方便后续做些效果用
    body->SetUserData(sprite);
}

void Mybox2dWorld::onTouchesBegan(const std::vector<Touch*>& touches, Event* event)
{
    //Add a new body/atlas sprite at the touched location
    
    for (auto& touch : touches)
    {
        if(!touch)
            break;
        
        _randomIdx = (CCRANDOM_0_1() > .5 ? 0:1);
        _randomIdy = (CCRANDOM_0_1() > .5 ? 0:1);
        
        auto location = touch->getLocation();
        _beganPos = location;
        CCLOG("Mybox2dWorld::onTouchesBegan(%.1f,%.1f)", location.x, location.y);
        _movedPos = _beganPos;
        _beganSprite->setVisible(true);
        _beganSprite->setPosition(_beganPos);
        _beganSprite->stopAllActions();
        
        // _beganSprite->runAction(RepeatForever::create(FadeTo::create(0.5f, 180)));
        _beganSprite->runAction(RepeatForever::create(Sequence::create(FadeTo::create(0.5f, 100), FadeTo::create(0.5f, 255), nullptr)));
        
    }
}

void Mybox2dWorld::onTouchesMoved(const std::vector<Touch*>& touches, Event *unused_event)
{
    for (auto& touch : touches)
    {
        if(!touch)
            break;
        
        _movedPos = touch->getLocation();
        
        if (_beganSprite->isVisible())
        {// 计算显示的角度
            Vec2 n = _movedPos - _beganPos;
            float a = CC_RADIANS_TO_DEGREES(n.getAngle(Vec2::UNIT_X)) + 180.0f;
            float b = CC_DEGREES_TO_RADIANS(a);
            float c = M_PI - n.getAngle(Vec2::UNIT_X);
            // 注意setRotation传入的是角度值
            _beganSprite->setRotation(a);
            CCLOG("Mybox2dWorld::onTouchesMoved(%.1f,%.1f) a=%.3f b=%.3f c=%.2f", _movedPos.x, _movedPos.y, a, b, c);
        }// if
        
        
    }
}

void Mybox2dWorld::onTouchesEnded(const std::vector<Touch*>& touches, Event* event)
{
    //Add a new body/atlas sprite at the touched location
    
    for (auto& touch : touches)
    {
        if(!touch)
            break;
        
        auto location = touch->getLocation();
        _movedPos = location;
        
        _beganSprite->stopAllActions();
        _beganSprite->setVisible(false);
        if ((_beganPos.x >= 0.0f) && (_beganPos.x <= VisibleRect::getVisibleRect().size.width) &&
            (_beganPos.y >= 0.0f) && (_beganPos.y <= VisibleRect::getVisibleRect().size.height))
        {// 简单判断下在显示范围内才增加
            addNewSpriteAtPosition( _beganPos );
        }// if
        
        CCLOG("Mybox2dWorld::onTouchesEnded(%.1f,%.1f) count=%d", location.x, location.y, (int)_blockParent->getChildrenCount());
    }
}

void Mybox2dWorld::draw(Renderer *renderer, const Mat4& transform, uint32_t flags)
{
    // 直接在draw里绘制不出来
    /*
    Director* director = Director::getInstance();
    CCASSERT(nullptr != director, "Director is null when seting matrix stack");
    director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    cocos2d::Mat4 t = transform;
    // t.translate(VisibleRect::center().x, VisibleRect::center().y, 0);
    t.scale(PTM_RATIO);
    director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, t);
    // director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, transform);
    
    GL::enableVertexAttribs( cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION );
    
    // _debugDraw.DrawSolidCircle(b2Vec2(50.0f, 50.0f), 10, b2Vec2(1.0f,1.0f), b2Color(0.5f, 0.5f, 0.0f));
    _world->DrawDebugData();
    
    CHECK_GL_ERROR_DEBUG();
    
    director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    */
    
    _customCmd.init(_globalZOrder, transform, flags);
    _customCmd.func = CC_CALLBACK_0(Mybox2dWorld::onDraw, this, transform, flags);
    renderer->addCommand(&_customCmd);
    
}

void Mybox2dWorld::onDraw(const cocos2d::Mat4& transform, uint32_t flags)
{
    Director* director = Director::getInstance();
    CCASSERT(nullptr != director, "Director is null when seting matrix stack");
    director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    cocos2d::Mat4 t = transform;
    // t.translate(VisibleRect::center().x, VisibleRect::center().y, 0);
    t.scale(PTM_RATIO);// box2dworld的反向缩放回来
    director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, t);
    // director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, transform);
    
    GL::enableVertexAttribs( cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION );
    
    // _debugDraw.DrawSolidCircle(b2Vec2(50.0f, 50.0f), 10, b2Vec2(1.0f,1.0f), b2Color(0.5f, 0.5f, 0.0f));
    
    if (_beganSprite && _beganSprite->isVisible())
    {
        _debugDraw.DrawSegment(b2Vec2(_beganPos.x / PTM_RATIO, _beganPos.y / PTM_RATIO),
                               b2Vec2(_movedPos.x / PTM_RATIO, _movedPos.y / PTM_RATIO),
                               b2Color(1.0f, 0.0f, 0.0f));
        
        // 计算力的方向
        Vec2 v = _movedPos - _beganPos;
        v.normalize();
        v = v * 30.0f;
        //  绿色，显示力的方向
        _debugDraw.DrawSegment(b2Vec2(_beganPos.x / PTM_RATIO, _beganPos.y / PTM_RATIO),
                               b2Vec2((_beganPos.x - v.x) / PTM_RATIO, (_beganPos.y - v.y) / PTM_RATIO),
                               b2Color(0.0f, 1.0f, 0.0f));
    }// if
    

    
    _world->DrawDebugData();
    
    CHECK_GL_ERROR_DEBUG();
    
    director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}

