/*
 * Copyright (c) 2013, others
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_DISPLAY_H
#define POINT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <geometry_msgs/PointStamped.h>
#include <rviz/message_filter_display.h>


namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class ColorProperty;
    class FloatProperty;
    class IntProperty;
}

namespace rviz
{

    class PointStampedVisual;

    class PointStampedDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PointStamped>
    {
    Q_OBJECT
    public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	PointStampedDisplay();
	virtual ~PointStampedDisplay();

    protected:
	// Overrides of public virtual functions from the Display class.
	virtual void onInitialize();
	virtual void reset();

     private Q_SLOTS:
	// Helper function to apply color and alpha to all visuals.
	void updateColorAndAlpha();
        void updateHistoryLength();

	// Function to handle an incoming ROS message.
    private:
	void processMessage( const geometry_msgs::PointStamped::ConstPtr& msg );

        // Storage for the list of visuals.  It is a circular buffer where
        // data gets popped from the front (oldest) and pushed to the back (newest)
        boost::circular_buffer<boost::shared_ptr<PointStampedVisual> > visuals_;

	// Property objects for user-editable properties.
	rviz::ColorProperty *color_property_;
        rviz::FloatProperty *alpha_property_, *radius_property_;
	rviz::IntProperty *history_length_property_;

    };
} // end namespace rviz_plugin_tutorials

#endif // POINT_DISPLAY_H
