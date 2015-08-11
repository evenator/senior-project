#ifdef MACOSX
#include <GLUT/glut.h>
#else
#include <glut.h>
#endif
#include "app.h" 

static int win;
void idle(void);
void render(void);
int ticks=0;
int main( int argc, char* argv[] )
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  // define the size
  glutInitWindowSize(VIDEO_WIDTH,VIDEO_HEIGHT);
  
  // the position where the window will appear
  glutInitWindowPosition(100,100);
  win = glutCreateWindow("QuadroCopterView");
  
	appInit();
  glutDisplayFunc(render);
  glutIdleFunc(idle);
  glClearColor(0.0,1.0,0.0,0.0);
  glutMainLoop();
	return 0;
}

void idle(void)
{
  glutPostRedisplay();
};

void render(void)
{
  ticks++;
  appRender(ticks, VIDEO_WIDTH, VIDEO_HEIGHT);
};