/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#version 330 core

in vec2 texcoord;

out vec4 FragColor;

uniform sampler2D rSampler;
uniform mat4 pose;
uniform vec4 cam; //cx, cy, 1/fx, 1/fy
uniform float cols;
uniform float rows;
uniform int first;

#include "geometry.glsl"

void main()
{
    //Should be guaranteed to be in bounds
    float x = texcoord.x * cols;
    float y = texcoord.y * rows;

    //Filtered position ONLY used for normal and radius calculation
    vec3 vPosition = getVertex(texcoord.xy, x, y, cam, rSampler);

    vec3 vNormLocal = getNormal(vPosition.xyz, texcoord.xy, x, y, cam, rSampler);

    if(first==0)
    {
	FragColor = vec4(vNormLocal, 1);
    }
    else{
        FragColor = vec4(mat3(pose)*vNormLocal, 1);
    }
   
 }
