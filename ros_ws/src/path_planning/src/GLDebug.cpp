#include "../include/GLDebug.h"
#include "../include/Map.h"
#include "../include/Node.h"
#include "../include/LocalOp.h"
#include "ros/ros.h"
#include "GL/freeglut.h"
#include "GL/gl.h"
#include <memory>


GLDebug::GLDebug() {

}

GLDebug::~GLDebug() {

}

void GLDebug::drawMap(std::shared_ptr<Map> m) {
    std::shared_ptr<Node> n = m->end;
    glColor3f(0.0, 1.0, 0.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    while(n) {
        glBegin(GL_POLYGON);
        glVertex2f(1020 - n->y_index * 20, 1020 - n->x_index * 20); 
        glVertex2f(1000 - n->y_index * 20, 1020 - n->x_index * 20); 
        glVertex2f(1000 - n->y_index * 20, 1000 - n->x_index * 20); 
        glVertex2f(1020 - n->y_index * 20, 1000 - n->x_index * 20); 
        glEnd();
        glColor3f(1.0, 1.0, 1.0);
        n = n->prevNode;
    }
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        for(unsigned int j = 0; j < MAP_SIZE; j++) {
            if(m->obstacle_grid(i, j) > 0) {
                glBegin(GL_POLYGON);
                glColor3f((float)m->obstacle_grid(i, j) / 128.0f, 0.0f, 0.0f);
                glVertex2f(1020 - j * 20, 1020 - i * 20); 
                glVertex2f(1000 - j * 20, 1020 - i * 20); 
                glVertex2f(1000 - j * 20, 1000 - i * 20); 
                glVertex2f(1020 - j * 20, 1000 - i * 20); 
                glEnd();
            }
        }
    }
}

void GLDebug::init(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(MAP_SIZE * 20, MAP_SIZE * 20);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("A*");
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, MAP_SIZE * 20, MAP_SIZE * 20, 0);
    glutDisplayFunc(GLDebug::disp);
    glutIdleFunc(GLDebug::idle);
}

void GLDebug::disp() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawMap(LocalOp::m);
    glBegin(GL_LINES);
    glLineWidth(1);
    glColor3f(1.0, 1.0, 1.0);
    for(unsigned int i = 0; i < MAP_SIZE; i++) {
        glVertex2d(i * 20, 0);
        glVertex2d(i * 20, MAP_SIZE * 20);
        glVertex2d(0, i * 20);
        glVertex2d(MAP_SIZE * 20, i * 20);
    }
    glEnd();
    glFlush();
    glutSwapBuffers();
}

void GLDebug::idle() {
    ros::spinOnce();
    glutPostRedisplay();
}
