/*
LICENSE

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.

FEEDBACK & QUESTIONS

For feedback and questions about stmbl please e-mail one of the authors named in
the AUTHORS file.
*/

#include <include/glwidget.hpp>

#include <QVector>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QtGlobal>

#include <math.h>

static const char *vertexShaderSource =
				"#version 120\n"
				"uniform mat4 mvp;\n"
				"void main(void)\n"
				"{\n"
				"    gl_Position = mvp * gl_Vertex;\n"
				"}\n";

static const char *fragmentShaderSource =
				"#version 120\n"
				"void main(void)\n"
				"{\n"
				"    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
				"}\n";

GLWidget::GLWidget(QWidget * parent) :
		QOpenGLWidget(parent)
{
	m_translating = false;
	m_scaling = false;

    m_center.setX(0.0f);
    m_center.setY(0.0f);
    m_scale = 1.0f;
	
	updateMatrix();
}

void GLWidget::resetMatrix()
{
    m_center.setX(0.0f);
    m_center.setY(0.0f);
    m_scale = 1.0f;

	repaint();
}

void GLWidget::updateMatrix()
{
	m_matrix.setToIdentity();
	m_matrix.ortho(-width()/2, +width()/2, +height()/2, -height()/2, 0.0f, 15.0f);

    m_matrix.scale(m_scale, m_scale, 1.0);
    m_matrix.translate(QVector3D(m_center.x(), m_center.y(), -10.0f));
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	m_vao.create();
	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

	m_vbo.create();
	m_vbo.bind();

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	if(f) {
		f->glEnableVertexAttribArray(0);
#ifdef __APPLE__
        //wat
        //http://stackoverflow.com/questions/28156524/meaning-of-index-parameter-in-glenablevertexattribarray-and-possibly-a-bug-i
        f->glEnableVertexAttribArray(1);
#endif
        f->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
	} else {
		qWarning("couldn't get function context");
	}

	QVector<GLfloat> data;

	data.append(-10000000.0);
	data.append(0.0);
	data.append(10000000.0);
	data.append(0.0);

	data.append(0.0);
	data.append(-10000000.0);
	data.append(0.0);
	data.append(10000000.0);

	m_vbo.allocate(data.constData(), data.count() * sizeof(GLfloat));

	m_vao.release();
	m_vbo.release();

	m_function1.initializeGL();

	m_shader = new QOpenGLShaderProgram();
	m_shader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
	m_shader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
	m_shader->link();
	m_shader->bind();
	m_shader->enableAttributeArray("vertex");
	m_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 2, 0);

	m_shader->release();
}

void GLWidget::paintGL()
{
	//monoclock::time_point t = monoclock::now();

	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	m_shader->bind();

	updateMatrix();

	m_shader->setUniformValue("mvp", m_matrix);
	m_shader->enableAttributeArray("vertex");
	m_shader->setAttributeBuffer("vertex", GL_FLOAT, 0, 2, 0);

	m_function1.paintGL();

	QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);

	QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
	if(f) {
		f->glDrawArrays(GL_LINES, 0, 4);
	}

	m_vao.release();

	m_shader->release();

	update();

	//monoclock::time_point t2 = monoclock::now();
	//auto seconds_passed = std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1, 1> > >(t2 - t).count();
	//qDebug() << 1 / seconds_passed;
}

void GLWidget::resizeGL(int w, int h)
{
    //int side = qMin(w, h);
    //glViewport((w - side) / 2, (h - side) / 2, side, side);
	updateMatrix();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	if(event->buttons() == Qt::LeftButton) {
		m_translating = true;
        m_translation_start = event->pos();
        //QVector3D p(event->pos().x() - width()/2, event->pos().y() - height()/2, 0.0);
        //qDebug() << event->pos() << " " << p << " " << p / m_scalation;
	}
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    //QPoint numPixels = event->pixelDelta();
    //QPoint numDegrees = event->angleDelta();
    //qDebug() << "wh pix " << numPixels << endl << "wh deg" << numDegrees;

    QVector2D mouse = QVector2D(event->pos().x() - width() / 2, event->pos().y() - height() / 2);
    m_center -= mouse / m_scale;
    m_scale *= 1.0f + event->angleDelta().y() / 500.0f; // TODO: angleDelta????
    m_center += mouse / m_scale;

    event->accept();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
	if(m_translating && event->buttons() == Qt::LeftButton) {
        QPoint pos = event->pos() - m_translation_start;
        m_translation_start = event->pos();
        m_center += QVector2D(pos) / m_scale;
	}
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
	if(event->buttons() != Qt::LeftButton)
		m_translating = false;

	if(event->buttons() != Qt::RightButton)
		m_scaling = false;
}
