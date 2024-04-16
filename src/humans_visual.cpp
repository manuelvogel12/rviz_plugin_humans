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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <geometry_msgs/Point.h>

#include "humans_visual.h"

namespace rviz_plugin_humans
{

HumansVisual::HumansVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  for(int i=0; i<number_lines; i++){
    bone_line_[i].reset(new rviz::Line( scene_manager_, frame_node_));
    bone_billboard_line_[i].reset(new rviz::BillboardLine( scene_manager_, frame_node_));
  }
  text_label = new rviz::MovableText( "NO ID","Liberation Sans", 0.2, Ogre::ColourValue(0.1,0.1,0.1,0.5));
  text_label->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
  frame_node_->attachObject(text_label);
}

HumansVisual::~HumansVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void HumansVisual::setMessage( const concert_msgs::Human3D& human )
{
  std::vector<std::pair<int,int>> arrow_pairs = {
    {24,11}, // chest
    {11,14}, {14,15}, {15,16}, // left arm
    {11,19}, {19,20}, {20,21}, // right arm
    {24,0}, {0,1}, {1,2}, {2,3}, // right leg
    {24,5}, {5,6}, {6,7}, {7,8}, // left leg
    {11,12} //head
    };

  // visualize bones
  for(int i=0; i<number_lines; i++)
  {
    const geometry_msgs::Point& start = human.keypoints[arrow_pairs[i].first].pose.position;
    const geometry_msgs::Point& end = human.keypoints[arrow_pairs[i].second].pose.position;

    // Convert the geometry_msgs::Vector3 to an Ogre::Vector3.
    Ogre::Vector3 start_vec( start.x, start.y, start.z );
    Ogre::Vector3 end_vec( end.x, end.y, end.z );

    // use lines and billboard lines since lines have no width, but billboard lines can be invisible 
    bone_line_[i]->setPoints(start_vec, end_vec);

    bone_billboard_line_[i]->addPoint(start_vec);
    bone_billboard_line_[i]->addPoint(end_vec);
    bone_billboard_line_[i]->setLineWidth(0.03);
  }
  
  //text label for Human ID
  text_label->setCaption("ID: " + std::to_string(human.label_id));
  // set the position for the label text
  const geometry_msgs::Point& human_origin = human.keypoints[11].pose.position;
  Ogre::Vector3 origin_vec( human_origin.x, human_origin.y, human_origin.z );
  text_label->setGlobalTranslation( origin_vec );
}

// Position and orientation are passed through to the SceneNode.
void HumansVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void HumansVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void HumansVisual::setColor( float r, float g, float b, float a )
{
  for(int i=0; i<number_lines; i++){
    //bone_arrow_[i]->setColor( r, g, b, a );
    //bone_line_[i]->setColor( r, g, b, a );
    bone_billboard_line_[i]->setColor( r, g, b, a );
  }
}

} // end namespace rviz_plugin_humans

