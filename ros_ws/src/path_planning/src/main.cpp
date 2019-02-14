/*******************************************************************************
 *  @package path_planning
 *  @brief Path planning code. Subscribes to the state output of the Kalman filter and
 *  outputs an optimal path using A*.
 *  @author Luke Armbruster 
 * 
 ******************************************************************************/

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "../include/Map.h"
#include "../include/Node.h"
#include "GL/freeglut.h"
#include "GL/gl.h"
#include <thread>

ros::Subscriber state_sub;
Map* m;

static void pathCallback(const std_msgs::String::ConstPtr& msg) {
    int x_index;
    int y_index;
    Map current_map(x_index, y_index);
}

void disp() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m->DrawMap();
    glBegin(GL_LINES);
    glLineWidth(1);
    glColor3f(1.0, 1.0, 1.0);
    for(unsigned int i = 0; i < 51; i++) {
        glVertex2d(i * 20, 0);
        glVertex2d(i * 20, MAP_SIZE * 20);
        glVertex2d(0, i * 20);
        glVertex2d(MAP_SIZE * 20, i * 20);
    }
    glEnd();
    glFlush();
    glutSwapBuffers();
}

void idle() {
    ros::spinOnce();
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    ros::NodeHandle nh;
    state_sub = nh.subscribe<std_msgs::String>("state", 100, pathCallback);

    /* glut init stuff */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(MAP_SIZE * 20, MAP_SIZE * 20);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("A*");
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, MAP_SIZE * 20, MAP_SIZE * 20, 0);
    glutDisplayFunc(disp);
    glutIdleFunc(idle);

    m = new Map(50, 25);
    std::shared_ptr<Node> n = m->AStarSearch();
    
    glutMainLoop();

    return 0;
}