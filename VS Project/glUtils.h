#pragma once
// ----------------------------------------------------------
// Funzione di richiamo specialKeys()
// ----------------------------------------------------------



void glutIdle(void)
{
	Sleep(20);
	glutPostRedisplay();
}


void initGL() {

	if (0) {
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
		glClearDepth(1.0f);                   // Set background depth to farthest
		glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
		glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
		glShadeModel(GL_SMOOTH);   // Enable smooth shading
		//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
		gluPerspective(100.0, (double)640 / (double)480, 0, 300.0);
	}
	else {

		GLfloat width = 640;
		GLfloat height = 480;
		GLfloat aspect = (GLfloat)width / (GLfloat)height;
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
		glClearDepth(1.0f);                   // Set background depth to farthest
		glutInitWindowPosition(0, 0);


		//glMatrixMode(GL_PROJECTION);
		glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
		glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
		glShadeModel(GL_SMOOTH);   // Enable smooth shading		gluPerspective(100, aspect, 0.2, 20.2);
		glMatrixMode(GL_MODELVIEW);
		//glViewport(0, 0, width, height);  //Use the whole window for rendering

	}

}





void reshape(GLsizei width, GLsizei height) {

	if (0) {

		// GLsizei for non-negative integer
												   // Compute aspect ratio of the new window
		if (height == 0) height = 1;                // To prevent divide by 0
		GLfloat aspect = (GLfloat)width / (GLfloat)height;

		// Set the viewport to cover the new window
		glViewport(0, 0, width, height);

		// Set the aspect ratio of the clipping volume to match the viewport
		glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
		glLoadIdentity();             // Reset
									  // Enable perspective projection with fovy, aspect, zNear and zFar
		//gluPerspective(45.0f, aspect, 0.1f, 100.0f);

	}

	else {
		if (height == 0 || width == 0) return;

		GLfloat aspect = (GLfloat)width / (GLfloat)height;

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(39.0, aspect, 0.6, 21.0);
		glMatrixMode(GL_MODELVIEW);
		glViewport(0, 0, width, height);  //Use the whole window for rendering

	}

}


/************************** draw_cylinder() **************************
* This function will draw the cylinder
*
*   @parameter1: radius = The radius of cylinder
*   @parameter2: height = Height of the cylinder
*   @parameter3: R = Red value of the cylinder's color
*   @parameter4: G = Green value of the cylinder's color
*   @parameter5: B = Blue value of the cylinder's color
*
*   @return: Nothing
*/
void draw_cylinder(GLfloat radius, GLfloat height, GLubyte R, GLubyte G, GLubyte B, GLfloat startX, GLfloat startY, GLfloat startZ)
{
	GLfloat x = 0.0;
	GLfloat y = 0.0;
	GLfloat z = 0.0;

	GLfloat angle = 0.0;
	GLfloat angle_stepsize = 0.1;

	/** Draw the tube */
	glColor3ub(R - 40, G - 40, B - 40);
	glBegin(GL_QUAD_STRIP);
	angle = 0.0;
	while (angle < 2 * M_PI) {
		x = radius * cos(angle) + startX;
		y = radius * sin(angle) + startY;

		glVertex3f(x, y, height + startZ);
		glVertex3f(x, y, 0.0 + startZ);
		angle = angle + angle_stepsize;
	}
	glVertex3f(radius, 0.0, height + startZ);
	glVertex3f(radius, 0.0, 0.0 + startZ);
	glEnd();

	/** Draw the circle on top of cylinder */
	glColor3ub(R, G, B);
	glBegin(GL_POLYGON);
	angle = 0.0;
	while (angle < 2 * M_PI) {
		x = radius * cos(angle) + startX;
		y = radius * sin(angle) + startY;
		glVertex3f(x, y, height + startZ);
		angle = angle + angle_stepsize;
	}
	glVertex3f(radius, 0.0, height + startZ);
	glEnd();
}



void drawCylinder(GLfloat radius, GLfloat halfLength, int slices)
{
	glColor3f(1.0, 0.0, 0.0);

	for (int i = 0; i < slices; i++) {
		float theta = ((float)i)*2.0*M_PI;
		float nextTheta = ((float)i + 1)*2.0*M_PI;
		glBegin(GL_TRIANGLE_STRIP);
		/*vertex at middle of end */ glVertex3f(0.0, halfLength, 0.0);
		/*vertices at edges of circle*/ glVertex3f(radius*cos(theta), halfLength, radius*sin(theta));
		glVertex3f(radius*cos(nextTheta), halfLength, radius*sin(nextTheta));
		/* the same vertices at the bottom of the cylinder*/
		glVertex3f(radius*cos(nextTheta), -halfLength, radius*sin(nextTheta));
		glVertex3f(radius*cos(theta), -halfLength, radius*sin(theta));
		glVertex3f(0.0, -halfLength, 0.0);
		glEnd();
	}
}

void cylinder(GLfloat startX, GLfloat startY, GLfloat startZ)
{
#define DEF_D 5
#define Cos(th) cos(M_PI/180*(th))
#define Sin(th) sin(M_PI/180*(th))
	float height = 2;
	/*  sides */
	glBegin(GL_QUAD_STRIP);
	for (int j = 0; j <= 360; j += DEF_D) {
		glColor3f(1.0, 1.0, 0.0);
		glVertex3f(Cos(j) + startX, startY, Sin(j) + startZ);
		//glColor3f(0.0, 1.0, 0.0);
		glVertex3f(Cos(j) + startX, startY + height, Sin(j) + startZ);
	}
	glEnd();

	/* top and bottom circles */
	/* reuse the currentTexture on top and bottom) */
	for (int i = 0; i < 2; i++) {
		glBegin(GL_TRIANGLE_FAN);
		glColor3f(0.0, 0.0, 1.0);
		// glVertex3f(0, i*height, 0);
		for (int k = 0; k <= 360; k += DEF_D) {
			//glColor3f(1.0, 0.0, 0.0);
			glVertex3f(i*Cos(k) + startX, startY + i * height, Sin(k) + startZ);
		}
		glEnd();
	}
}



void draw_sphere(GLfloat X, GLfloat Y, GLfloat Z, GLfloat radius)
{

	glPushMatrix();
	glTranslatef(X, Y, Z);
	glColor3f(1.0, 1.0, 0.0);
	glutSolidSphere(radius, 50, 50);

	glPopMatrix();

}