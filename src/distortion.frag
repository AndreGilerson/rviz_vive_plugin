#version 400

uniform sampler2D InputTexture;
uniform sampler2D DistortionTextureRed;
uniform sampler2D DistortionTextureGreen;
uniform sampler2D DistortionTextureBlue;

varying vec2 Texcoord;

void main( void )
{
	vec4 redCoord = texture2D(DistortionTextureRed, Texcoord);
	vec4 greenCoord = texture2D(DistortionTextureGreen, Texcoord);
	vec4 blueCoord = texture2D(DistortionTextureBlue, Texcoord);
	
	vec2 dR = vec2(redCoord.x, redCoord.y);
	vec2 dG = vec2(greenCoord.x, greenCoord.y);
	vec2 dB = vec2(blueCoord.x, blueCoord.y);
	
	vec4 r = texture2D(InputTexture, dR);
	vec4 g = texture2D(InputTexture, dG);
	vec4 b = texture2D(InputTexture, dB);
	
	gl_FragColor = vec4(r.x, g.y, b.z, 1.0);}