#ifndef GBUFFER_H
#define	GBUFFER_H

#include <GL/glew.h>

#define GBUFFER_DEPTH_TEXTURE_UNIT 0
#define GBUFFER_POSITION_TEXTURE_UNIT 1

class GBuffer
{
public:

	enum GBUFFER_TEXTURE_TYPE {
		GBUFFER_TEXTURE_TYPE_DEPTH,
		GBUFFER_TEXTURE_TYPE_COLOR,
		GBUFFER_NUM_TEXTURES
	};

	GBuffer();

	~GBuffer();

	bool init(unsigned int WindowWidth, unsigned int WindowHeight);

	bool initNew(unsigned int WindowWidth, unsigned int WindowHeight);

	void bindForWriting();

	void bindForReading();

	void setReadBuffer(GBUFFER_TEXTURE_TYPE TextureType);

private:

	GLuint m_fbo;
	GLuint m_textures[GBUFFER_NUM_TEXTURES];
	GLuint m_depthTexture;
	GLuint m_colorTexture1;
	GLuint m_colorTexture2;
};

#endif	/* SHADOWMAPFBO_H */

