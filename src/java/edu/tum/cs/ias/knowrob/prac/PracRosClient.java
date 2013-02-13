/*
* Copyright (C) 2012 by Moritz Tenorth
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

package edu.tum.cs.ias.knowrob.prac;

import java.util.ArrayList;
import ros.*;

import ros.pkg.rosprac.msg.action_role;
import ros.pkg.rosprac.srv.pracinfer;


/**
* Service client for the PRAC inference tool
*
* @author Moritz Tenorth, tenorth@cs.uni-bremen.de
*/
public class PracRosClient {

  static Boolean rosInitialized = false;
  static Ros ros;
  static NodeHandle n;


  /**
  * Call the pracinfer service and return the result
  *
  * @return An ArrayList of action_role
  */
  public static ArrayList<action_role> callPracInferService(String action_core, String instruction) {

    initRos("knowrob_tutorial_client");
    ArrayList<action_role> roles = null;
    try {

      // call the dummy_object_detector service
      pracinfer.Request req = new pracinfer.Request();
      req.action_core = action_core;
      req.instruction = instruction;

      ServiceClient<pracinfer.Request, pracinfer.Response, pracinfer> cl =
      n.serviceClient("/pracinfer", new pracinfer());

      roles = cl.call(req).roles;
      cl.shutdown();

    } catch (RosException e) {
      ros.logError("ROSClient: Call to service /pracinfer failed");
    }
    return roles;
  }


  /**
  * Initialize the ROS environment if it has not yet been initialized
  *
  * @param node_name A unique node name
  */
  protected static void initRos(String node_name) {

    ros = Ros.getInstance();

    if(!Ros.getInstance().isInitialized()) {
      ros.init(node_name);
    }
    n = ros.createNodeHandle();
  }
}