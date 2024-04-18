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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "humans_visual.h"
#include "humans_display.h"

namespace rviz_plugin_humans
{


HumansDisplay::HumansDisplay()
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

}


void HumansDisplay::onInitialize()
{
  MFDClass::onInitialize();
  ros::NodeHandle nh;
  _humans_in_scene_sub = nh.subscribe("/sara_shield/humans_in_scene", 100, &HumansDisplay::humansInSceneCallback, this);

}

HumansDisplay::~HumansDisplay()
{
}

// Clear the visuals by deleting their objects.
void HumansDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void HumansDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
  }
}


// This is our callback to handle an incoming message.
void HumansDisplay::processMessage( const concert_msgs::Humans::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  
  // TODO: remove as soon as source_frame is specified by profactor
  std::string source_frame = msg->header.frame_id;
  if (source_frame==""){
    source_frame="base_link";
  }
  //std::cout<<"source frame"<< source_frame<<std::endl;
  if( !context_->getFrameManager()->getTransform( source_frame,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
              source_frame.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  for(const concert_msgs::Human3D& human: msg->humans)
  {
    int label_id = human.label_id;
    if(label_id >= 0)
    {
      boost::shared_ptr<HumansVisual> visual;
      
      if (label_id >= visuals_.size())
      {
        visuals_.resize(label_id + 1);
      }

      visual.reset(new HumansVisual( context_->getSceneManager(), scene_node_ ));

      // Now set or update the contents of the chosen visual.
      visual->setMessage( human );
      visual->setFramePosition( position );
      visual->setFrameOrientation( orientation );

      float alpha = alpha_property_->getFloat();
      Ogre::ColourValue color = color_property_->getOgreColor();
      visual->setColor( color.r, color.g, color.b, alpha );

      // And send it to the end of the circular buffer
      visuals_[label_id] =  visual;
    }
  }
}

// remove all humans visuals
void HumansDisplay::humansInSceneCallback(const std_msgs::Bool& msg) {
  if (!msg.data) {
    reset();
  }
}

} // end namespace rviz_plugin_humans

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_humans::HumansDisplay,rviz::Display )


