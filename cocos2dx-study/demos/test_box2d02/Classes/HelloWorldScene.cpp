#include "HelloWorldScene.h"
#include "AppMacros.h"

HelloWorld::~HelloWorld()
{
    _myWorld = nullptr;
}

Scene* HelloWorld::scene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    HelloWorld *layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    auto visibleSize = Director::getInstance()->getVisibleSize();
    auto origin = Director::getInstance()->getVisibleOrigin();

    /////////////////////////////
    // 2. add a menu item with "X" image, which is clicked to quit the program
    //    you may modify it.

    // add a "close" icon to exit the progress. it's an autorelease object
    auto closeItem = MenuItemImage::create(
                                        "CloseNormal.png",
                                        "CloseSelected.png",
                                        CC_CALLBACK_1(HelloWorld::menuCloseCallback,this));
    
    closeItem->setPosition(origin + Vec2(visibleSize) - Vec2(closeItem->getContentSize() / 2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, nullptr);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);
    
    /////////////////////////////
    // 3. add your codes below...

    // add a label shows "Hello World"
    // create and initialize a label
    
    //auto label = Label::createWithTTF("测试Box2d", "fonts/arial.ttf", TITLE_FONT_SIZE);
    // auto label = Label::createWithSystemFont("测试Box2d", "微软雅黑", TITLE_FONT_SIZE);
    auto label = Label::createWithSystemFont("测试Box2d02", "微软雅黑", 12);
    label->setTextColor(Color4B::BLUE);
    // enableOutline在mac os上无效，手机上是可以的
    // label->enableOutline(Color4B::GREEN, 2);
    
    label->enableShadow(Color4B::GRAY, Size(1.0f, -1.0f));
    
    Director* director = Director::getInstance();
    label->setString(StringUtils::format("测试Box2d02 WinSize(%.1f,%.1f) WinSizeInPixels(%.1f, %.1f) %.2f", director->getWinSize().width, director->getWinSize().height, director->getWinSizeInPixels().width, director->getWinSizeInPixels().height, director->getContentScaleFactor()));
    
    // position the label on the center of the screen
    label->setPosition(origin.x + visibleSize.width/2,
                            origin.y + visibleSize.height - label->getContentSize().height);

    // add the label as a child to this layer
    this->addChild(label, 1);

    // add "HelloWorld" splash screen"
    auto sprite = Sprite::create("HelloWorld.png");

    // position the sprite on the center of the screen
    sprite->setPosition(Vec2(visibleSize / 2) + origin);

    // add the sprite as a child to this layer
    this->addChild(sprite);

    _myWorld = Mybox2dWorld::create();    
    this->addChild(_myWorld);
    _myWorld->setPosition(0.0f, 0.0f);
    
    return true;
}

void HelloWorld::menuCloseCallback(Ref* sender)
{
    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}

