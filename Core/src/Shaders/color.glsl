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
vec3 lcolorType[30]=vec3[30]
		(vec3(0.33,0,0.12),
                   vec3(0.1,0,0),
                   vec3(0,0.1,0),
                   vec3(0,0,0.1),
                   vec3(0.1,0.1,0),
                   vec3(0,0.1,0.1),
                   vec3(0.1,0,0.1),
                   vec3(0.2,0,0),
                   vec3(0,0.2,0),
                   vec3(0,0,0.2),
		vec3(0.2,0,0.2),
                   vec3(0.2,0.2,0),
                   vec3(0,0.2,0.2),
                   vec3(0.3,0,0),
                   vec3(0,0.3,0),
                   vec3(0,0,0.3),
                   vec3(0.4,0.4,0),
                   vec3(0,0.4,0.4),
                   vec3(0.4,0,0.4),
                   vec3(0.7,0,0),
		vec3(0,0.7,0),
                   vec3(0,0,0.7),
                   vec3(0.7,0.7,0),
                   vec3(0,0.7,0.7),
                   vec3(0.7,0,0.7),
                   vec3(0.8,0,0),
                   vec3(0,0.8,0),
                   vec3(0,0,0.9),
                   vec3(0,0.8,0.8),
                   vec3(0.8,0.8,0));
float encodeColor(vec3 c)
{
    int rgb = int(round(c.x * 255.0f));
    rgb = (rgb << 8) + int(round(c.y * 255.0f));
    rgb = (rgb << 8) + int(round(c.z * 255.0f));
    return float(rgb);
}

vec3 decodeColor(float c)
{
    vec3 col;
    col.x = float(int(c) >> 16 & 0xFF) / 255.0f;
    col.y = float(int(c) >> 8 & 0xFF) / 255.0f;
    col.z = float(int(c) & 0xFF) / 255.0f;
    return col;
}


