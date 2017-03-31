/*
 * 分割DEPTH MAP
 */

float getlabel(vec2 texCoord,  sampler2D depth)
{
    float z = float((int)(textureLod(depth, texCoord, 0.0))%255);

    return z;
}

