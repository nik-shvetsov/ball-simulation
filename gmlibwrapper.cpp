#include "gmlibwrapper.h"
#include "glcontextsurfacewrapper.h"

#include "testtorus.h"
#include "utils.h"
#include "gmpbiplane.h"
#include "gmpcurplane.h"
#include "gmpwall.h"
#include "ball.h"
#include "collision.h"
#include "controller.h"

// GMlib
#include <gmOpenglModule>
#include <gmSceneModule>
#include <gmParametricsModule>

// Qt
#include <QTimerEvent>
#include <QRectF>
#include <QMouseEvent>
#include <QDebug>

// stl
#include <stdexcept>
#include <thread>
#include <mutex>





std::unique_ptr<GMlibWrapper> GMlibWrapper::_instance {nullptr};


GMlibWrapper::GMlibWrapper(std::shared_ptr<GLContextSurfaceWrapper> context)
//  : GMlibWrapper()
  : QObject(), _timer_id{0}, _glsurface(context), _select_renderer{nullptr}
{

  if(_instance != nullptr) {

    std::cerr << "This version of the GMlibWrapper only supports a single instance of the GMlibWraper..." << std::endl;
    std::cerr << "Only one of me(0x" << this << ") please!! ^^" << std::endl;
    assert(!_instance);
    exit(666);
  }


  _instance = std::unique_ptr<GMlibWrapper>(this);

  _glsurface->makeCurrent();

  // Setup and initialized GMlib GL backend
  GMlib::GL::OpenGLManager::init();

  // Setup and init the GMlib GMWindow
  _scene = std::make_shared<GMlib::Scene>();
}

GMlibWrapper::~GMlibWrapper() {

  stop();

  _glsurface->makeCurrent(); {

    _select_renderer->releaseCamera();
    _select_renderer.reset();

    for( auto& rc_pair : _rc_pairs ) {

      rc_pair.second.render->releaseCamera();
      _scene->removeCamera( rc_pair.second.camera.get() );

      rc_pair.second.render.reset();
      rc_pair.second.camera.reset();
    }

    _scene->clear();

  } _glsurface->doneCurrent();
}

void GMlibWrapper::changeRenderGeometry(const QString& name, const QRectF& geometry) {

  const QSize size = geometry.size().toSize();

  if( size.width() <= 0 || size.height() <= 0 )
    return;

  if( _rc_pairs.count(name.toStdString()) <= 0 )
    return;

  auto& rc_pair = _rc_pairs[name.toStdString()];
  if(rc_pair.viewport.geometry == geometry )
    return;

  rc_pair.viewport.geometry = geometry;
  rc_pair.viewport.changed = true;
}

void GMlibWrapper::timerEvent(QTimerEvent* e) {

  e->accept();

  // Simuation order
  // 1) Prepare must be run first
  // 2) Simulate and render can be run in parallell


  // Grab and activate GL context
  _glsurface->makeCurrent(); {

    // 1)
    _scene->prepare();


    _scene->simulate();

//    std::vector<std::thread> threads;

    // Add simulation thread
//    threads.push_back(std::thread(&GMlib::Scene::simulate,_scene));

    // Add Render threads
    for( auto& rc_pair : _rc_pairs ) {
  //      qDebug() << "About to render: " << rc_pair.first.c_str();
  //      qDebug() << "  Viewport: ";
  //      qDebug() << "    Changed: " << rc_pair.second.viewport.changed;
  //      qDebug() << "    Geometry: " << rc_pair.second.viewport.geometry;

      if(rc_pair.second.viewport.changed) {
        const QSizeF size = rc_pair.second.viewport.geometry.size();
        rc_pair.second.render->reshape( GMlib::Vector<int,2>(size.width(),size.height()));
        rc_pair.second.camera->reshape( 0, 0, size.width(), size.height() );
        rc_pair.second.viewport.changed = false;
      }

      rc_pair.second.render->render();
      rc_pair.second.render->swap();
    }

//    for( auto& thread : threads )
//      thread.join();

  } _glsurface->doneCurrent();

  emit signFrameReady();
}

const GMlibWrapper&
GMlibWrapper::getInstance() {
  return *_instance;
}

void GMlibWrapper::start() {

  if( _timer_id || _scene->isRunning() )
    return;

  _timer_id = startTimer(16, Qt::PreciseTimer);
  _scene->start();
}

void GMlibWrapper::stop() {

  if( !_timer_id || !_scene->isRunning() )
    return;

  _scene->stop();
  killTimer(_timer_id);
  _timer_id = 0;
}

void GMlibWrapper::initScene() {

  // Make OpenGL context current on offscreensurface
  _glsurface->makeCurrent(); {

    // Insert a light
    GMlib::Point<GLfloat,3> init_light_pos( 2.0, 4.0, 10 );
    GMlib::PointLight *light = new GMlib::PointLight(  GMlib::GMcolor::White, GMlib::GMcolor::White,
                                                       GMlib::GMcolor::White, init_light_pos );
    light->setAttenuation(0.8, 0.002, 0.0008);
    _scene->insertLight( light, false );

    // Insert Sun
    _scene->insertSun();


    int init_viewport_size = 600;
    GMlib::Point<float,3> init_cam_pos(  0.0f, 0.0f, 0.0f );
    GMlib::Vector<float,3> init_cam_dir( 0.0f, 1.0f, 0.0f );
    GMlib::Vector<float,3> init_cam_up(  0.0f, 0.0f, 1.0f );

    _rc_pairs.reserve(4);
    _rc_pairs["Projection"] = RenderCamPair {};
    _rc_pairs["Front"]      = RenderCamPair {};
    _rc_pairs["Side"]       = RenderCamPair {};
    _rc_pairs["Top"]        = RenderCamPair {};

    for( auto& rcpair : _rc_pairs ) {

      rcpair.second.render = std::make_shared<GMlib::DefaultRenderer>();
      rcpair.second.camera = std::make_shared<GMlib::Camera>();
      rcpair.second.render->setCamera(rcpair.second.camera.get());
    }

    // Projection cam
    auto& proj_rcpair = _rc_pairs["Projection"];
    proj_rcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
    proj_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
    proj_rcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
    proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, -30.0f, 30.0f ) );
    _scene->insertCamera( proj_rcpair.camera.get() );
    proj_rcpair.render->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );

    // Front cam
    auto& front_rcpair = _rc_pairs["Front"];
    front_rcpair.camera->set( init_cam_pos + GMlib::Vector<float,3>( 0.0f, -50.0f, 0.0f ), init_cam_dir, init_cam_up );
    front_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
    _scene->insertCamera( front_rcpair.camera.get() );
    front_rcpair.render->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );

    // Side cam
    auto& side_rcpair = _rc_pairs["Side"];
    side_rcpair.camera->set( init_cam_pos + GMlib::Vector<float,3>( -50.0f, 0.0f, 0.0f ), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ), init_cam_up );
    side_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
    _scene->insertCamera( side_rcpair.camera.get() );
    side_rcpair.render->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );

    // Top cam
    auto& top_rcpair = _rc_pairs["Top"];
    top_rcpair.camera->set( init_cam_pos + GMlib::Vector<float,3>( 0.0f, 0.0f, 50.0f ), -init_cam_up, init_cam_dir );
    top_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
    _scene->insertCamera( top_rcpair.camera.get() );
    top_rcpair.render->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );





//    // Iso Camera
//    auto& isorcpair = (_rc_pairs["Iso"] = RenderCamPair {});
//    isorcpair.render = std::make_shared<GMlib::DefaultRenderer>();
//    isorcpair.camera = std::make_shared<GMlib::IsoCamera>();
//    isorcpair.render->setCamera(isorcpair.camera.get());
//    _scene->insertCamera( isorcpair.camera.get() );
//    isorcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
//    isorcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
//    isorcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
//    isorcpair.camera->translate( GMlib::Vector<float,3>( 0.0f, -20.0f, 20.0f ) );
//    isorcpair.render->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );


    // Setup Select Renderer
    _select_renderer = std::make_shared<GMlib::DefaultSelectRenderer>();



//#define TEST_CURVE
#define TEST_SURFACE


#ifdef TEST_CURVE

    // Curve visualizers
    auto curve_visualizer = new GMlib::PCurveDerivativesVisualizer<float,3>;

    // Curve
    auto curve = new GMlib::PCircle<float>(2.0f);
    curve->toggleDefaultVisualizer();
    curve->insertVisualizer(curve_visualizer);
    curve->replot(100,1);
    _scene->insert(curve);

#endif





#ifdef TEST_SURFACE

    // Surface visualizers
//    auto surface_visualizer = new GMlib::PSurfDerivativesVisualizer<float,3>;
//    auto surface_visualizer = new GMlib::PSurfNormalsVisualizer<float,3>;
    auto surface_visualizer = new GMlib::PSurfParamLinesVisualizer<float,3>;
//   auto surface_visualizer = new GMlib::PSurfPointsVisualizer<float,3>;

    //surface->insertVisualizer(surface_visualizer);
    // Surface test torus
    //surface->test01();



      //adding a ball
//    auto object1 = new Ball(1);
//    //auto object1 = new Ball(1,1,GMlib::Vector<float,3>(1,0,0),GMlib::Point<float,3>(-7,0,1));
//    object1->toggleDefaultVisualizer();
//    //object1->translate(GMlib::Vector<float,3>(-7,0,1));
//    object1->replot(200,200,1,1);
//    //object properties
//    object1->setMaterial(GMlib::GMmaterial::Silver);
//    object1->setMass(5);
//    object1->setVelocity(GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ));
//    object1->setLocation(GMlib::Point<float,3>(-7,0,1));
//    //object1->setVelocity(GMlib::Vector<float,3>( 0.0f, 0.0f, 0.0f ));
//    _scene->insert(object1);


      //adding new seashell
//    //GMlib::PSeashell<float>* shell = new GMlib::PSeashell<float>();
//    auto shell = new GMlib::PSphere<float>;
//    shell->toggleDefaultVisualizer();
//    shell->replot(200,200,1,1);
//    shell->translate(GMlib::Vector<float,3>(0,0,0));
//    shell->setMaterial(GMlib::GMmaterial::Bronze);
//    _scene->insert(shell);
//    //biplane->insert(shell);


      //adding new biplane of 4 points
//    //PBIPlane<float>*
//    auto biplane = new PBiPlane<float>
//            (GMlib::Point<float,3>(0,0,0), GMlib::Point<float,3>(2,0,0),
//             GMlib::Point<float,3>(2,0,3), GMlib::Point<float,3>(1,1,5));
//    biplane->toggleDefaultVisualizer();
//    biplane->replot(30,30,1,1);
//    biplane->setMaterial(GMlib::GMmaterial::Gold);
//    _scene->insert(biplane);


      //adding curved plane
//    GMlib::DMatrix<GMlib::Vector<float,3>> m(3,3);
//    m[0][0] = GMlib::Vector<float,3> (1,0,0);
//    m[0][1] = GMlib::Vector<float,3> (2,0,0);
//    m[0][2] = GMlib::Vector<float,3> (3,0,0);
//    m[1][0] = GMlib::Vector<float,3> (0,1,0);
//    m[1][1] = GMlib::Vector<float,3> (0,2,0);
//    m[1][2] = GMlib::Vector<float,3> (0,3,0);
//    m[2][0] = GMlib::Vector<float,3> (0,4,1);
//    m[2][1] = GMlib::Vector<float,3> (0,5,2);
//    m[2][2] = GMlib::Vector<float,3> (0,6,3);

//    auto pcurved = new PCurPlane<float>(m);
//    pcurved->toggleDefaultVisualizer();
//    pcurved->replot(200,200,1,1);
//    _scene->insert(pcurved);


      //creating box of walls
//    //PBIPlane<float>*
//    auto wallN = new PWall<float>
//                (GMlib::Point<float,3>(-10,8,2), GMlib::Point<float,3>(-10,8,0),
//                 GMlib::Point<float,3>(-10,-8,0), GMlib::Point<float,3>(-10,-8,2));
//    wallN->toggleDefaultVisualizer();
//    wallN->replot(30,30,1,1);
//    wallN->setMaterial(GMlib::GMmaterial::Gold);
//    _scene->insert(wallN);


//    auto wallS = new PWall<float>
//                (GMlib::Point<float,3>(10,8,2), GMlib::Point<float,3>(10,-8,2),
//                 GMlib::Point<float,3>(10,-8,0), GMlib::Point<float,3>(10,8,0));
//    wallS->toggleDefaultVisualizer();
//    wallS->replot(30,30,1,1);
//    wallS->setMaterial(GMlib::GMmaterial::Gold);
//    _scene->insert(wallS);

//    auto wallW = new PWall<float>
//                (GMlib::Point<float,3>(-10,8,2), GMlib::Point<float,3>(10,8,2),
//                 GMlib::Point<float,3>(10,8,0), GMlib::Point<float,3>(-10,8,0));
//    wallW->toggleDefaultVisualizer();
//    wallW->replot(30,30,1,1);
//    wallW->setMaterial(GMlib::GMmaterial::Gold);
//    _scene->insert(wallW);

//    auto wallE = new PWall<float>
//                (GMlib::Point<float,3>(-10,-8,2), GMlib::Point<float,3>(10,-8,2),
//                 GMlib::Point<float,3>(10,-8,0), GMlib::Point<float,3>(-10,-8,0));
//    wallE->toggleDefaultVisualizer();
//    wallE->replot(30,30,1,1);
//    wallE->setMaterial(GMlib::GMmaterial::Gold);
//    _scene->insert(wallE);


      //creating floor surface
//    auto floor = new PBiPlane<float>
//                (GMlib::Point<float,3>(-10,8,0), GMlib::Point<float,3>(10,8,0),
//                 GMlib::Point<float,3>(10,-8,0), GMlib::Point<float,3>(-10,-8,0));
//    floor->toggleDefaultVisualizer();
//    floor->replot(30,30,1,1);
//    floor->setMaterial(GMlib::GMmaterial::Jade);
//    _scene->insert(floor);

      //creating balls
//    auto ball1 = new Ball(1,1,GMlib::Vector<float,3>(1,0,0),GMlib::Point<float,3>(-7,0,1), floor);
//    ball1->insertVisualizer(surface_visualizer);
//    ball1->replot(200,200,1,1);
//    ball1->setMaterial(GMlib::GMmaterial::Ruby);
//    _scene->insert(ball1);


//    auto ball2 = new Ball(5,1,GMlib::Vector<float,3>(-1,0,0),GMlib::Point<float,3>(0,0,0), floor);
//    ball2->toggleDefaultVisualizer();
//    ball2->replot(200,200,1,1);
//    ball2->setMaterial(GMlib::GMmaterial::Sapphire);
//    _scene->insert(ball2);


    //with collision controller-----------------------------------------------------------------------
    //simplewalls with getnormal
           auto wallN = new PWall
                   (GMlib::Point<float,3>(10,10,0), GMlib::Vector<float,3>(0,0,2), GMlib::Vector<float,3>(-20,0,0));

           auto wallS = new PWall
                   (GMlib::Point<float,3>(-10,-10,0), GMlib::Vector<float,3>(0,0,2), GMlib::Vector<float,3>(20,0,0));

           auto wallW = new PWall
                   (GMlib::Point<float,3>(10,-10,0), GMlib::Vector<float,3>(0,0,2), GMlib::Vector<float,3>(0,20,0));

           auto wallE = new PWall
                   (GMlib::Point<float,3>(-10,10,0), GMlib::Vector<float,3>(0,0,2), GMlib::Vector<float,3>(0,-20,0));

//           auto wallN = new PWall<float>
//                           (GMlib::Point<float,3>(-10,8,4), GMlib::Point<float,3>(-10,8,0),
//                            GMlib::Point<float,3>(-10,-8,0), GMlib::Point<float,3>(-10,-8,4));
//           auto wallS = new PWall<float>
//                           (GMlib::Point<float,3>(10,8,4), GMlib::Point<float,3>(10,-8,4),
//                            GMlib::Point<float,3>(10,-8,0), GMlib::Point<float,3>(10,8,0));
//           auto wallW = new PWall<float>
//                           (GMlib::Point<float,3>(-10,8,4), GMlib::Point<float,3>(10,8,4),
//                            GMlib::Point<float,3>(10,8,0), GMlib::Point<float,3>(-10,8,0));
//           auto wallE = new PWall<float>
//                           (GMlib::Point<float,3>(-10,-8,4), GMlib::Point<float,3>(10,-8,4),
//                            GMlib::Point<float,3>(10,-8,0), GMlib::Point<float,3>(-10,-8,0));



//           auto floor = new PBiPlane<float>
//                   (GMlib::Point<float,3>(-10,10,0), GMlib::Point<float,3>(10,10,0),
//                    GMlib::Point<float,3>(10,-10,0), GMlib::Point<float,3>(-10,-10,0));
//           floor->toggleDefaultVisualizer();
//           floor->replot(30,30,1,1);
//           //floor->setMaterial(GMlib::GMmaterial::Jade);

           //---------------------curved test
               //adding curved plane

               GMlib::DMatrix<GMlib::Vector<float,3>> m(11,11);

               m[0][0] = GMlib::Vector<float,3> (-10,-10,0);
               m[0][1] = GMlib::Vector<float,3> (-8,-10,0);
               m[0][2] = GMlib::Vector<float,3> (-6,-10,0);
               m[0][3] = GMlib::Vector<float,3> (-4,-10,0);
               m[0][4] = GMlib::Vector<float,3> (-2,-10,0);
               m[0][5] = GMlib::Vector<float,3> (0,-10,0);
               m[0][6] = GMlib::Vector<float,3> (2,-10,0);
               m[0][7] = GMlib::Vector<float,3> (4,-10,0);
               m[0][8] = GMlib::Vector<float,3> (6,-10,0);
               m[0][9] = GMlib::Vector<float,3> (8,-10,0);
               m[0][10] = GMlib::Vector<float,3> (10,-10,0);

               m[1][0] = GMlib::Vector<float,3> (-10,-8,0);
               m[1][1] = GMlib::Vector<float,3> (-8,-8,0);
               m[1][2] = GMlib::Vector<float,3> (-6,-8,0);
               m[1][3] = GMlib::Vector<float,3> (-4,-8,0);
               m[1][4] = GMlib::Vector<float,3> (-2,-8,0);
               m[1][5] = GMlib::Vector<float,3> (0,-8,0);
               m[1][6] = GMlib::Vector<float,3> (2,-8,0);
               m[1][7] = GMlib::Vector<float,3> (4,-8,0);
               m[1][8] = GMlib::Vector<float,3> (6,-8,0);
               m[1][9] = GMlib::Vector<float,3> (8,-8,0);
               m[1][10] = GMlib::Vector<float,3> (10,-8,0);

               m[2][0] = GMlib::Vector<float,3> (-10,-6,0);
               m[2][1] = GMlib::Vector<float,3> (-8,-6,0);
               m[2][2] = GMlib::Vector<float,3> (-6,-6,0);
               m[2][3] = GMlib::Vector<float,3> (-4,-6,0);
               m[2][4] = GMlib::Vector<float,3> (-2,-6,0);
               m[2][5] = GMlib::Vector<float,3> (0,-6,0);
               m[2][6] = GMlib::Vector<float,3> (2,-6,0);
               m[2][7] = GMlib::Vector<float,3> (4,-6,0);
               m[2][8] = GMlib::Vector<float,3> (6,-6,0);
               m[2][9] = GMlib::Vector<float,3> (8,-6,0);
               m[2][10] = GMlib::Vector<float,3> (10,-6,0);

               m[3][0] = GMlib::Vector<float,3> (-10,-4,0);
               m[3][1] = GMlib::Vector<float,3> (-8,-4,0);
               m[3][2] = GMlib::Vector<float,3> (-6,-4,0);
               m[3][3] = GMlib::Vector<float,3> (-4,-4,0);
               m[3][4] = GMlib::Vector<float,3> (-2,-4,0);
               m[3][5] = GMlib::Vector<float,3> (0,-4,0);
               m[3][6] = GMlib::Vector<float,3> (2,-4,0);
               m[3][7] = GMlib::Vector<float,3> (4,-4,0);
               m[3][8] = GMlib::Vector<float,3> (6,-4,0);
               m[3][9] = GMlib::Vector<float,3> (8,-4,0);
               m[3][10] = GMlib::Vector<float,3> (10,-4,0);

               m[4][0] = GMlib::Vector<float,3> (-10,-2,0);
               m[4][1] = GMlib::Vector<float,3> (-8,-2,0);
               m[4][2] = GMlib::Vector<float,3> (-6,-2,0);
               m[4][3] = GMlib::Vector<float,3> (-4,-2,0);
               m[4][4] = GMlib::Vector<float,3> (-2,-2,0);
               m[4][5] = GMlib::Vector<float,3> (0,-2,0);
               m[4][6] = GMlib::Vector<float,3> (2,-2,0);
               m[4][7] = GMlib::Vector<float,3> (4,-2,0);
               m[4][8] = GMlib::Vector<float,3> (6,-2,0);
               m[4][9] = GMlib::Vector<float,3> (8,-2,0);
               m[4][10] = GMlib::Vector<float,3> (10,-2,0);

               m[5][0] = GMlib::Vector<float,3> (-10,0,0);
               m[5][1] = GMlib::Vector<float,3> (-8,0,0);
               m[5][2] = GMlib::Vector<float,3> (-6,0,0);
               m[5][3] = GMlib::Vector<float,3> (-4,0,0);
               m[5][4] = GMlib::Vector<float,3> (-2,0,0);
               m[5][5] = GMlib::Vector<float,3> (0,0,100);
               m[5][6] = GMlib::Vector<float,3> (2,0,0);
               m[5][7] = GMlib::Vector<float,3> (4,0,0);
               m[5][8] = GMlib::Vector<float,3> (6,0,0);
               m[5][9] = GMlib::Vector<float,3> (8,0,0);
               m[5][10] = GMlib::Vector<float,3> (10,0,0);

               m[6][0] = GMlib::Vector<float,3> (-10,2,0);
               m[6][1] = GMlib::Vector<float,3> (-8,2,0);
               m[6][2] = GMlib::Vector<float,3> (-6,2,0);
               m[6][3] = GMlib::Vector<float,3> (-4,2,0);
               m[6][4] = GMlib::Vector<float,3> (-2,2,0);
               m[6][5] = GMlib::Vector<float,3> (0,2,0);
               m[6][6] = GMlib::Vector<float,3> (2,2,0);
               m[6][7] = GMlib::Vector<float,3> (4,2,0);
               m[6][8] = GMlib::Vector<float,3> (6,2,0);
               m[6][9] = GMlib::Vector<float,3> (8,2,0);
               m[6][10] = GMlib::Vector<float,3> (10,2,0);

               m[7][0] = GMlib::Vector<float,3> (-10,4,0);
               m[7][1] = GMlib::Vector<float,3> (-8,4,0);
               m[7][2] = GMlib::Vector<float,3> (-6,4,0);
               m[7][3] = GMlib::Vector<float,3> (-4,4,0);
               m[7][4] = GMlib::Vector<float,3> (-2,4,0);
               m[7][5] = GMlib::Vector<float,3> (0,4,0);
               m[7][6] = GMlib::Vector<float,3> (2,4,0);
               m[7][7] = GMlib::Vector<float,3> (4,4,0);
               m[7][8] = GMlib::Vector<float,3> (6,4,0);
               m[7][9] = GMlib::Vector<float,3> (8,4,0);
               m[7][10] = GMlib::Vector<float,3> (10,4,0);

               m[8][0] = GMlib::Vector<float,3> (-10,6,0);
               m[8][1] = GMlib::Vector<float,3> (-8,6,0);
               m[8][2] = GMlib::Vector<float,3> (-6,6,0);
               m[8][3] = GMlib::Vector<float,3> (-4,6,0);
               m[8][4] = GMlib::Vector<float,3> (-2,6,0);
               m[8][5] = GMlib::Vector<float,3> (0,6,0);
               m[8][6] = GMlib::Vector<float,3> (2,6,0);
               m[8][7] = GMlib::Vector<float,3> (4,6,0);
               m[8][8] = GMlib::Vector<float,3> (6,6,0);
               m[8][9] = GMlib::Vector<float,3> (8,6,0);
               m[8][10] = GMlib::Vector<float,3> (10,6,0);

               m[9][0] = GMlib::Vector<float,3> (-10,8,0);
               m[9][1] = GMlib::Vector<float,3> (-8,8,0);
               m[9][2] = GMlib::Vector<float,3> (-6,8,0);
               m[9][3] = GMlib::Vector<float,3> (-4,8,0);
               m[9][4] = GMlib::Vector<float,3> (-2,8,0);
               m[9][5] = GMlib::Vector<float,3> (0,8,0);
               m[9][6] = GMlib::Vector<float,3> (2,8,0);
               m[9][7] = GMlib::Vector<float,3> (4,8,0);
               m[9][8] = GMlib::Vector<float,3> (6,8,0);
               m[9][9] = GMlib::Vector<float,3> (8,8,0);
               m[9][10] = GMlib::Vector<float,3> (10,8,0);

               m[10][0] = GMlib::Vector<float,3> (-10,10,0);
               m[10][1] = GMlib::Vector<float,3> (-8,10,0);
               m[10][2] = GMlib::Vector<float,3> (-6,10,0);
               m[10][3] = GMlib::Vector<float,3> (-4,10,0);
               m[10][4] = GMlib::Vector<float,3> (-2,10,0);
               m[10][5] = GMlib::Vector<float,3> (0,10,0);
               m[10][6] = GMlib::Vector<float,3> (2,10,0);
               m[10][7] = GMlib::Vector<float,3> (4,10,0);
               m[10][8] = GMlib::Vector<float,3> (6,10,0);
               m[10][9] = GMlib::Vector<float,3> (8,10,0);
               m[10][10] = GMlib::Vector<float,3> (10,10,0);

               auto floor = new GMlib::PBezierSurf<float>(m);
               floor->toggleDefaultVisualizer();
               floor->replot(40,40,1,1);
               //floor->setMaterial(GMlib::GMmaterial::Jade);

           //-------------------------


           floor->getDefaultVisualizer()->setDisplayMode(GMlib::Visualizer::DISPLAY_MODE_WIREFRAME);
           //proj_rcpair.camera->lock(floor);

           auto colController = new Controller(floor);
           _scene->insert(colController);


           //test balls----------------------------------------
           auto ball1 = new Ball(1,5,GMlib::Vector<float,3>(5,0,0), floor);
           colController->insertBall(ball1);
           ball1->insertVisualizer(surface_visualizer);
           ball1->replot(100,100,1,1);
           ball1->setMaterial(GMlib::GMmaterial::Obsidian);
           ball1->translate(GMlib::Point<float,3>(-8,0,1));


           auto ball2 = new Ball(1,5,GMlib::Vector<float,3>(-5,0,0), floor);
           colController->insertBall(ball2);
           ball2->insertVisualizer(surface_visualizer);
           ball2->replot(100,100,1,1);
           ball2->setMaterial(GMlib::GMmaterial::Ruby);
           ball2->translate(GMlib::Point<float,3>(8,0,1));


            //player controlled ball
           _contrBall = new Ball(1,5,GMlib::Vector<float,3>(0,5,0), floor);
           colController->insertBall(_contrBall);
           _contrBall->insertVisualizer(surface_visualizer);
           _contrBall->replot(100,100,1,1);
           _contrBall->setMaterial(GMlib::GMmaterial::Emerald);
           _contrBall->translate(GMlib::Point<float,3>(0,5,1));

           //-------------------------------------------------


           wallN->toggleDefaultVisualizer();
           wallN->replot(30,30,1,1);
           wallN->setMaterial(GMlib::GMmaterial::Gold);
           colController->insertWall(wallN);

           wallS->toggleDefaultVisualizer();
           wallS->replot(30,30,1,1);
           wallS->setMaterial(GMlib::GMmaterial::Gold);
           colController->insertWall(wallS);

           wallE->toggleDefaultVisualizer();
           wallE->replot(30,30,1,1);
           wallE->setMaterial(GMlib::GMmaterial::Gold);
           colController->insertWall(wallE);

           wallW->toggleDefaultVisualizer();
           wallW->replot(30,30,1,1);
           wallW->setMaterial(GMlib::GMmaterial::Gold);
           colController->insertWall(wallW);

#endif

  } _glsurface->doneCurrent();
}

const std::shared_ptr<GMlib::Scene>&
GMlibWrapper::getScene() const {

  return _scene;
}

const GMlib::TextureRenderTarget&
GMlibWrapper::getRenderTextureOf(const std::string& name) const {

  if(!_rc_pairs.count(name)) throw std::invalid_argument("[][]Render/Camera pair '" + name + "'  does not exist!");

  return _rc_pairs.at(name).render->getFrontRenderTarget();
}

void
GMlibWrapper::mousePressed(const QString& name, QMouseEvent* event ) {

    /*///////////////////////////////////////////////////////////////
    /// \brief GMlibWrapper::mousePressed
    /// \param name  - the name of the camera
    /// \param event - the mouse event, using position and butten
    ///////////////////////////////////////////////////////////////*/

    const QPointF& pos = event->pos();

        _current_mouse_pos = _prev_mouse_pos = GMlib::Point<int,2>(pos.x(),pos.y());

        const auto& rc_select = _rc_pairs.at(name.toStdString());
        const auto& rc_geo    =  rc_select.viewport.geometry;

        auto& rc   = _rc_pairs.at(name.toStdString());
        auto cam   =  rc.camera.get();

        if(event->button() == Qt::RightButton)
        {
          if(_rotate_object_button_pressed)         // Ctrl
          {
            GMlib::SceneObject * sel_obj = getScene()->getSelectedObjects()(0);

            qDebug() << "Ctrl pressed" << sel_obj;

            if(sel_obj)
            {
              cam->lock(sel_obj);
            }
            else
            {
              if(cam->isLocked()) cam->unLock();

              cam->lock((getScene()->getSphereClean().getPos() - cam->getPos())*cam->getDir());

              qDebug() << "Lock" << (getScene()->getSphereClean().getPos() - cam->getPos())*cam->getDir();
            }
          }
          else if(_move_object_button_pressed)      // Shift
          {
            qDebug() << "Shift pressed";
          }
          else if(_select_multiple_objects_pressed) // Alt
          {
            qDebug() << "Alt pressed";
            GMlib::SceneObject* obj{nullptr};

            GMlib::Vector<int,2> size(rc_geo.width(),rc_geo.height());
            _select_renderer->setCamera(cam);

            _glsurface->makeCurrent();
            {
              _select_renderer->reshape(size);
              _select_renderer->prepare();
              _select_renderer->select(0);
              obj = _select_renderer->findObject(pos.x(),size(1)-pos.y()-1);
            }
            _glsurface->doneCurrent();

            if(obj)
              obj->toggleSelected();
          }
          else
          {
            GMlib::SceneObject* obj{nullptr};
            _select_renderer->setCamera(cam);

            GMlib::Vector<int,2> size(rc_geo.width(),rc_geo.height());

            _glsurface->makeCurrent();
            {
              _select_renderer->reshape(size);
              _select_renderer->prepare();
              _select_renderer->select(0);
              obj = _select_renderer->findObject(pos.x(),size(1)-pos.y()-1);
            }
            _glsurface->doneCurrent();

            if(obj)
            {
//              if(obj == cam->getLockObj())
//                cam->unLock();
//              else
                cam->lock(obj);
            }
            else
            {
              if(cam->isLocked())
                cam->unLock();
              else
                cam->lock((getScene()->getSphereClean().getPos() - cam->getPos())*cam->getDir());
            }
          }
        }
        qDebug() << getScene()->getSelectedObjects().getSize();
}

void GMlibWrapper::mouseReleased(const QString& name, QMouseEvent* event) {
    Q_UNUSED(name)
    Q_UNUSED(event)
}

void GMlibWrapper::mouseDoubleClicked(const QString& name, QMouseEvent* event) {
    Q_UNUSED(name)
    Q_UNUSED(event)
}

void GMlibWrapper::mouseMoved(const QString &name, QMouseEvent *event)
{
    /*//////////////////////////////////////////////////////////////
    /// \brief GMlibWrapper::mouseMoved
    /// \param name  - the name of the camera
    /// \param event - the mouse event, using position and butten
    //////////////////////////////////////////////////////////////*/
        Q_UNUSED(name);
        Q_UNUSED(event);

        const QPointF& pos = event->pos();

        _current_mouse_pos = GMlib::Point<int,2>(pos.x(), pos.y());

        if(_prev_mouse_pos.getLength() <= 0.0f)
            _prev_mouse_pos = _current_mouse_pos;

        auto& rc    = _rc_pairs.at(name.toStdString());
        auto  cam   = rc.camera.get();
        float SNAP  = 0.01f;

        if(_move_object_button_pressed)
        {
            _glsurface->makeCurrent();

            const GMlib::Array<GMlib::SceneObject*> &sel_objs = getScene()->getSelectedObjects();

            for( int i = 0; i < sel_objs.getSize(); i++ ) {
                GMlib::SceneObject* obj = sel_objs(i);

                if( obj ) {
                    const double dh = cam->deltaTranslate( obj );
                    const GMlib::Vector<float,3> deltav(
                                ( ( _prev_mouse_pos(0) - _current_mouse_pos(0) ) * dh ) * cam->getSide() +
                                ( ( _prev_mouse_pos(1) - _current_mouse_pos(1) ) * dh ) * cam->getUp() );

                    if( deltav.getLength() > SNAP && deltav.getLength() < 1000.0 ) {
                        if( obj->getTypeId() != GMlib::GM_SO_TYPE_SELECTOR )
                            obj->translateGlobal( deltav );
                        else if( obj->getTypeId()== GMlib::GM_SO_TYPE_SELECTOR )
                            obj->editPos(deltav);
                    }
                }
            }
            _glsurface->doneCurrent();
        }
        else if(_rotate_object_button_pressed)
        {
            _glsurface->makeCurrent();

            const GMlib::Vector<int,2> pos = _current_mouse_pos;
            const GMlib::Vector<int,2> prev = _prev_mouse_pos;
            const GMlib::Array<GMlib::SceneObject*> &objs = getScene()->getSelectedObjects();

            // Compute rotation axis and angle in respect to the camera and view.
            const GMlib::UnitVector<float,3> rot_v =
                    float( pos(0) - prev(0) ) * cam->getUp() -
                    float( pos(1) - prev(1) ) * cam->getSide();

            const GMlib::Angle ang(
                    M_2PI * sqrt(
                       pow( double( pos(0) - prev(0) ) / cam->getViewportW(), 2 ) +
                       pow( double( prev(1) - pos(1) ) / cam->getViewportH(), 2 ) ) );

            int no_objs = 0;
            GMlib::Sphere<float,3> sphere;
            for( int i = 0; i < objs.getSize(); ++i )
                if( objs(i)->getTypeId() != GMlib::GM_SO_TYPE_SELECTOR ) {
                    sphere += objs(i)->getSurroundingSphereClean();
                    no_objs++;
                }

            for( int i = 0; i < objs.getSize(); ++i )

                if( objs(i)->getTypeId() != GMlib::GM_SO_TYPE_SELECTOR )
                    if( std::abs(pos(0)-prev(0)) > POS_TOLERANCE || std::abs(pos(1)-prev(1)) > POS_TOLERANCE )
                        no_objs > 1 ? objs(i)->rotate( ang, sphere.getPos(), rot_v) : objs(i)->rotate( ang, rot_v);

            _glsurface->doneCurrent();
        }
        else if(_select_multiple_objects_pressed) // Alt
        {

        }
        else
        {
          float scale;

          if(cam->isLocked()) scale = M_2PI * cam->getLockDist();
          else                scale = getScene()->getSphere().getRadius();

          const GMlib::Vector<float,2> delta (
                (_current_mouse_pos(0) - _prev_mouse_pos(0)) * scale / cam->getViewportW(),
                (_current_mouse_pos(1) - _prev_mouse_pos(1)) * scale / cam->getViewportH() );

          cam->move( delta );
        }

        _prev_mouse_pos = _current_mouse_pos;
}

void
GMlibWrapper::keyPressed(const QString& name, QKeyEvent* event) {
    Q_UNUSED(name)

    if( event->key() == Qt::Key_R ) _scene->toggleRun();

    if( event->key() == Qt::Key_E )
    {
        _glsurface->makeCurrent();

        const GMlib::Array<GMlib::SceneObject*> &sel_objs = getScene()->getSelectedObjects();
        for( int i = 0; i < sel_objs.getSize(); i++ ) {

            GMlib::SceneObject *sel_obj = sel_objs(i);

            // ERBS
            GMlib::PERBSSurf<float> *esObj = dynamic_cast<GMlib::PERBSSurf<float>*>( sel_obj );

            // Bezier
            GMlib::PBezierSurf<float> *bsObj = dynamic_cast<GMlib::PBezierSurf<float>*>( sel_obj );


            if( esObj ) {

                if( esObj->isLocalPatchesVisible() )
                    esObj->hideLocalPatches();
                else
                    esObj->showLocalPatches();

            }
            else if( bsObj ) {

                GMlib::PERBSSurf<float> *parent = dynamic_cast<GMlib::PERBSSurf<float>*>( bsObj->getParent());
                if( parent ) {

                    if( bsObj->toggleCollapsed() )
                        bsObj->hideSelectors();
                    else
                        bsObj->showSelectors(true);
                }
                else {

                    if( bsObj->isSelectorsVisible() )
                        bsObj->hideSelectors();
                    else
                        bsObj->showSelectors(true);
                }

            }
        }
        _glsurface->doneCurrent();
    }

    if(event->key() == Qt::Key_Shift)
    {
        _move_object_button_pressed = true;
    }

    if(event->key() == Qt::Key_Control)
    {
        _rotate_object_button_pressed = true;
    }

    if(event->key() == Qt::Key_Alt)
    {
        _select_multiple_objects_pressed = true;
    }

    if(event->key() == Qt::Key_W)
    {
        _glsurface->makeCurrent();

        const GMlib::Array<GMlib::SceneObject*> &sel_objs = getScene()->getSelectedObjects();
        for( int i = 0; i < sel_objs.getSize(); i++ ) {

            GMlib::SceneObject *sel_obj = sel_objs(i);

            // ERBS
            GMlib::PERBSSurf<float> *es_obj = dynamic_cast<GMlib::PERBSSurf<float>*>( sel_obj );

            if( es_obj ) {

              GMlib::Array<GMlib::Visualizer*> &visus = es_obj->getVisualizers();
              for( int i = 0; i < visus.getSize(); ++i ) {
                visus[i]->toggleDisplayMode();
              }
              es_obj->replot();
            }
        }
        _glsurface->doneCurrent();
    }

    if(event->key() == Qt::Key_P)
    {
        _glsurface->makeCurrent();

        const GMlib::Array<GMlib::SceneObject*> &sel_objs = getScene()->getSelectedObjects();
        for( int i = 0; i < sel_objs.getSize(); i++ ) {

            GMlib::SceneObject *sel_obj = sel_objs(i);

            // ERBS
            GMlib::PERBSSurf<float> *es_obj = dynamic_cast<GMlib::PERBSSurf<float>*>( sel_obj );
            _replot_low_medium_high += 2;

            if(_replot_low_medium_high>5)
                _replot_low_medium_high = 1;

            if( es_obj )
                es_obj->replot(_replot_low_medium_high*10,_replot_low_medium_high*10,2,2);
        }
        _glsurface->doneCurrent();
    }

    if(event->key() == Qt::Key_Up)
    {
        _contrBall->moveUp();
    }

    if(event->key() == Qt::Key_Down)
    {
        _contrBall->moveDown();
    }

    if(event->key() == Qt::Key_Left)
    {
        _contrBall->moveLeft();
    }

    if(event->key() == Qt::Key_Right)
    {
        _contrBall->moveRight();
    }
}

void GMlibWrapper::keyReleased(const QString& name, QKeyEvent* event) {
    Q_UNUSED(name)

    if(event->key() == Qt::Key_Shift)
        _move_object_button_pressed = false;
    if(event->key() == Qt::Key_Control)
        _rotate_object_button_pressed = false;
    if(event->key() == Qt::Key_Alt)
        _select_multiple_objects_pressed = false;
}

void GMlibWrapper::wheelEventOccurred(const QString& name, QWheelEvent* event) {

    int delta = event->delta();
    auto& rc = _rc_pairs.at(name.toStdString());

    const auto& camera_geo = rc.viewport.geometry;
    auto camera    = rc.camera.get();
    auto isocamera = dynamic_cast<GMlib::IsoCamera*>(camera);
    if(isocamera) {

        if( delta < 0 ) isocamera->zoom( 1.05 );
        if( delta > 0 ) isocamera->zoom( 0.95 );
    }
    else if(camera) {

        double scale;
        if( camera->isLocked() )
            scale = camera->getLockDist();
        else
            scale = getScene()->getSphere().getRadius();

        camera->move( delta*scale / camera_geo.height());

    }

}
