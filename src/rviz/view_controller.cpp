/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/load_resource.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/window_manager_interface.h"
#include "rviz/ogre_helpers/render_system.h"

#include "rviz/view_controller.h"

namespace rviz
{

ViewController::ViewController()
  : context_( NULL )
  , camera_( NULL )
  , is_active_( false )
  , type_property_( NULL )
  , avoid_stereo_update_( false )
{
  near_clip_property_ = new FloatProperty( "Near Clip Distance", 0.01f,
                                      "Anything closer to the camera than this threshold will not get rendered.",
                                      this, SLOT( updateNearClipDistance() ) );
  near_clip_property_->setMin( 0.001 );
  near_clip_property_->setMax( 10000 );

  stereo_enable_ = new BoolProperty( "Enable Stereo Rendering", true,
                                      "Render the main view in stereo if supported."
                                      "  On Linux this requires a recent version of Ogre and"
                                      " an NVIDIA Quadro card with 3DVision glasses.",
                                      this, SLOT( updateStereoProperties() ) );
  stereo_eye_swap_ = new BoolProperty( "Swap Stereo Eyes", false,
                                      "Swap eyes if the monitor shows the left eye on the right.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  autoset_stereo_properties_ = new BoolProperty( "Autoset stereo properties", true,
                                      "Set eye separation and focal distance automatically based on rviz focal point.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  window_dpi_ = new FloatProperty( "Window DPI", 81.0f,
                                      "Dots per inch on screen.  Used to autoset stereo properties."
                                      " Divide width_in_pixels over width_in_inches."
                                      " A 27inch 1920x1080 monitor is about 81dpi.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  physical_eye_separation_ = new FloatProperty( "Physical Eye Width", 0.05f,
                                      "Physical distance between viewer's eyes in the real world.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  physical_screen_distance_ = new FloatProperty( "Physical Screen Distance", 0.60f,
                                      "Physical distance from viewer to screen in the real world.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  stereo_eye_separation_ = new FloatProperty( "Stereo Eye Separation", 0.06f,
                                      "Distance between eyes for stereo rendering.  Set automatically"
                                      " if \"Autoset stereo properties\" is true.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
  stereo_focal_distance_ = new FloatProperty( "Stereo Focal Distance", 1.0f,
                                      "Distance from eyes to screen.  For stereo rendering.  Set"
                                      " automatically if \"Autoset stereo properties\" is true.",
                                      stereo_enable_, SLOT( updateStereoProperties() ), this );
}

void ViewController::initialize( DisplayContext* context )
{
  context_ = context;

  std::stringstream ss;
  static int count = 0;
  ss << "ViewControllerCamera" << count++;
  camera_ = context_->getSceneManager()->createCamera( ss.str() );
  context_->getSceneManager()->getRootSceneNode()->attachObject( camera_ );

  setValue( formatClassId( getClassId() ));
  setReadOnly( true );

  // Do subclass initialization.
  onInitialize();

  cursor_ = getDefaultCursor();

  standard_cursors_[Default] = getDefaultCursor();
  standard_cursors_[Rotate2D] = makeIconCursor( "package://rviz/icons/rotate.svg" );
  standard_cursors_[Rotate3D] = makeIconCursor( "package://rviz/icons/rotate_cam.svg" );
  standard_cursors_[MoveXY] = makeIconCursor( "package://rviz/icons/move2d.svg" );
  standard_cursors_[MoveZ] = makeIconCursor( "package://rviz/icons/move_z.svg" );
  standard_cursors_[Zoom] = makeIconCursor( "package://rviz/icons/zoom.svg" );
  standard_cursors_[Crosshair] = makeIconCursor( "package://rviz/icons/crosshair.svg" );

  updateNearClipDistance();
  updateStereoProperties();

  if (!RenderSystem::get()->isStereoSupported())
  {
    stereo_enable_->setBool(false);
    stereo_enable_->hide();
  }
}

ViewController::~ViewController()
{
  context_->getSceneManager()->destroyCamera( camera_ );
}

QString ViewController::formatClassId( const QString& class_id )
{
  QStringList id_parts = class_id.split( "/" );
  if( id_parts.size() != 2 )
  {
    // Should never happen with pluginlib class ids, which are
    // formatted like "package_name/class_name".  Not worth crashing
    // over though.
    return class_id;
  }
  else
  {
    return id_parts[ 1 ] + " (" + id_parts[ 0 ] + ")";
  }
}

QVariant ViewController::getViewData( int column, int role ) const
{
  if ( role == Qt::TextColorRole )
  {
    return QVariant();
  }

  if( is_active_ )
  {
    switch( role )
    {
    case Qt::BackgroundRole:
    {
      return QColor( 0xba, 0xad, 0xa4 );
    }
    case Qt::FontRole:
    {
      QFont font;
      font.setBold( true );
      return font;
    }
    }
  }
  return Property::getViewData( column, role );
}

Qt::ItemFlags ViewController::getViewFlags( int column ) const
{
  if( is_active_ )
  {
    return Property::getViewFlags( column );
  }
  else
  {
    return Property::getViewFlags( column ) | Qt::ItemIsDragEnabled;
  }
}

void ViewController::activate()
{
  is_active_ = true;
  onActivate();
}

void ViewController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void ViewController::load( const Config& config )
{
  // Load the name by hand.
  QString name;
  if( config.mapGetString( "Name", &name ))
  {
    setName( name );
  }
  // Load all sub-properties the same way the base class does.
  Property::load( config );
}

void ViewController::save( Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
  config.mapSetValue( "Name", getName() );

  Property::save( config );
}

void ViewController::handleKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  if( event->key() == Qt::Key_F &&
      panel->getViewport() &&
      context_->getSelectionManager() )
  {
    QPoint mouse_rel_panel = panel->mapFromGlobal( QCursor::pos() );
    Ogre::Vector3 point_rel_world; // output of get3DPoint().
    if( context_->getSelectionManager()->get3DPoint( panel->getViewport(),
                                                     mouse_rel_panel.x(), mouse_rel_panel.y(),
                                                     point_rel_world ))
    {
      lookAt( point_rel_world );
    }
  }

  if( event->key() == Qt::Key_Z )
  {
    reset();
  }
}

void ViewController::setCursor( CursorType cursor_type )
{
  cursor_=standard_cursors_[cursor_type];
}

void ViewController::lookAt( float x, float y, float z )
{
  Ogre::Vector3 point( x, y, z );
  lookAt( point );
}

void ViewController::lookAt( const Ogre::Vector3& point )
{
  setLookAt(point);
}

void ViewController::setLookAt( const Ogre::Vector3& point )
{
  look_at_[0] = point.x;
  look_at_[1] = point.y;
  look_at_[2] = point.z;
}

void ViewController::setStatus( const QString & message )
{
  if ( context_ )
  {
    context_->setStatus( message );
  }
}

void ViewController::updateNearClipDistance()
{
  float n = near_clip_property_->getFloat();
  camera_->setNearClipDistance( n );
}

void ViewController::updateStereoProperties()
{
  if (avoid_stereo_update_)
    return;

  if (stereo_enable_->getBool() &&
      camera_->getProjectionType() == Ogre::PT_PERSPECTIVE)
  {
#if 0
    // These are now set in updateCameraForStereoRendering()
    float focal_dist = stereo_focal_distance_->getFloat();
    float eye_sep = stereo_eye_swap_->getBool() ?
                    -stereo_eye_separation_->getFloat() :
                    stereo_eye_separation_->getFloat();
    camera_->setFrustumOffset(0.5f * eye_sep, 0.0f);
    camera_->setFocalLength(focal_dist);
#endif

    stereo_eye_swap_->show();
    stereo_eye_separation_->show();
    stereo_focal_distance_->show();
    autoset_stereo_properties_->show();
    if (autoset_stereo_properties_->getBool())
    {
      window_dpi_->show();
      physical_eye_separation_->show();
      physical_screen_distance_->show();
      stereo_eye_separation_->setReadOnly(true);
      stereo_focal_distance_->setReadOnly(true);
    }
    else
    {
      window_dpi_->hide();
      physical_eye_separation_->hide();
      physical_screen_distance_->hide();
      stereo_eye_separation_->setReadOnly(false);
      stereo_focal_distance_->setReadOnly(false);
    }
  }
  else
  {
    camera_->setFrustumOffset(0.0f,0.0f);
    camera_->setFocalLength(1.0f);
    stereo_eye_swap_->hide();
    autoset_stereo_properties_->hide();
    window_dpi_->hide();
    physical_eye_separation_->hide();
    physical_screen_distance_->hide();
    stereo_eye_separation_->hide();
    stereo_focal_distance_->hide();
  }
}

void ViewController::updateCameraForStereoRendering()
{
printf("stereo=%d  autoset=%d  viewport=%08lx\n",
stereo_enable_->getBool()?1:0,
autoset_stereo_properties_->getBool()?1:0,
(long)camera_->getViewport());

  if (!stereo_enable_->getBool())
    return;
  const Ogre::Viewport *viewport = camera_->getViewport();
  if (!autoset_stereo_properties_->getBool() ||
      !viewport)
  {
    float focal_dist = stereo_focal_distance_->getFloat();
    float eye_sep = stereo_eye_swap_->getBool() ?
                    -stereo_eye_separation_->getFloat() :
                    stereo_eye_separation_->getFloat();
    camera_->setFrustumOffset(0.5f * eye_sep, 0.0f);
    camera_->setFocalLength(focal_dist);
    return;
  }

  static const float INCH_PER_METER = 39.3701f;

  float phys_dpm = window_dpi_->getFloat() * INCH_PER_METER;
  float phys_eye_sep = physical_eye_separation_->getFloat();

  // TODO: try setting field of view based on physical screen distance?
  float phys_screen_dist = physical_screen_distance_->getFloat();

  const Ogre::Vector3& pos = camera_->getRealPosition();
  Ogre::Vector3 dir = camera_->getRealDirection();
  dir.normalise();

  // find new look_at_ that is on the look ray
  Ogre::Vector3 look_at(look_at_[0], look_at_[1], look_at_[2]);
  Ogre::Vector3 dir2 = look_at - pos;
  Ogre::Real dist = dir2.length();
  Ogre::Vector3 look_at2 = pos + (dir * dist);

  // Some view controllers may not call setLookAt(), so update look_at_ if
  // necessary.
  static const float mm = 0.001;
  if ((look_at2 - look_at).squaredLength() > mm*mm)
    setLookAt(look_at);

  // force focal length to be half way to object
  Ogre::Real focal_len_world = dist * 0.5f;

  Ogre::Radian fovy = camera_->getFOVy();
  Ogre::Real tanThetaX = Ogre::Math::Tan(camera_->getFOVy()) * camera_->getAspectRatio();
  Ogre::Real world_width_at_focal = tanThetaX * focal_len_world * 2.0f;

  Ogre::Real width_pixels = (Ogre::Real)viewport->getActualWidth();
  Ogre::Real width_phys_meters = width_pixels / phys_dpm;
  if (width_phys_meters <= std::numeric_limits<float>::epsilon())
    width_phys_meters = 0.30;
  Ogre::Real phys_to_world = world_width_at_focal / width_phys_meters;
  
  Ogre::Real eye_sep_world = phys_eye_sep / phys_to_world;


printf("dist=%7.3f fovy=%7.3f deg  tanThetaX=%7.3f world_width_at_focal=%7.3f  width_pixels=%7.3f  width_phys_meters=%7.3f phys_to_world=%7.3f eye_sep_world=%7.3f\n",
(double)dist,
(double)fovy.valueDegrees(),
(double)tanThetaX,
(double)world_width_at_focal,
(double)width_pixels,
(double)width_phys_meters,
(double)phys_to_world,
(double)eye_sep_world);

  avoid_stereo_update_ = true;
  if (phys_to_world > std::numeric_limits<float>::epsilon())
  {
    phys_screen_dist = focal_len_world / phys_to_world;
    physical_screen_distance_->setReadOnly(true);
    physical_screen_distance_->setValue(phys_screen_dist);
  }
  stereo_eye_separation_->setValue(eye_sep_world);
  stereo_focal_distance_->setValue(focal_len_world);
  avoid_stereo_update_ = false;
  float eye_sep = stereo_eye_swap_->getBool() ?
                  -eye_sep_world :
                  eye_sep_world;
  camera_->setFrustumOffset(0.5f * eye_sep, 0.0f);
  camera_->setFocalLength(focal_len_world);
}

} // end namespace rviz
