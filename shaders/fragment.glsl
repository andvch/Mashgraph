#version 330

#define float2 vec2
#define float3 vec3
#define float4 vec4
#define float4x4 mat4
#define float3x3 mat3

in float2 fragmentTexCoord;
layout(location = 0) out vec4 fragColor;

uniform int g_screenWidth;
uniform int g_screenHeight;
uniform mat4 g_rayMatrix;
uniform vec3 g_camPos;
uniform float fieldOfView;
uniform int mode;
uniform float iTime;
uniform samplerCube skybox;

vec2 iResolution = vec2(float(g_screenWidth), float(g_screenHeight));
vec2 fragCoord = fragmentTexCoord * iResolution;


bool b_shadow, b_fog;

void setMode(void) {
	
	if (mode == 0) {
		b_shadow = false;
		b_fog = false;
	} else if (mode == 1) {
		b_shadow = true;
		b_fog = false;
	} else if (mode == 2) {
		b_shadow = false;
		b_fog = true;
	} else if (mode == 3) {
		b_shadow = true;
		b_fog = true;
	}
	
}

//======================

const float EPSILON = 0.0001;
const int MAX_MARCHING_STEPS = 1000;
const float MIN_DIST = 0.01;
const float MAX_DIST = 100.0;

const int MAX_RAYS = 10;
const int RAY_BUF_SIZE = 2;
const int NUMBER_OF_OBJECTS = 8;


struct material {
	
	vec3 color;
	vec4 prop;	// x - reflection, y - light conductivity (x + y <= 1.0)
				// z - factor of refraction, w - shininess
	
} obj[NUMBER_OF_OBJECTS + 1];

struct rayInfo {
	
	vec3 p;		// ray start
	vec3 dir;	// 
	float d;	// share of total color
	int env;	// the propagation medium of light
	
};

mat3 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(1, 0, 0),
        vec3(0, c, -s),
        vec3(0, s, c)
    );
}

mat3 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, 0, s),
        vec3(0, 1, 0),
        vec3(-s, 0, c)
    );
}

mat3 rotateZ(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, -s, 0),
        vec3(s, c, 0),
        vec3(0, 0, 1)
    );
}

float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}


float boxSDF(vec3 p, vec3 size) {
    vec3 d = abs(p) - (size / 2.0);
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);
    float outsideDistance = length(max(d, 0.0));
    return insideDistance + outsideDistance;
}

float sphereSDF(vec3 p, float r) {
    return length(p) - r;
}

float coneSDF( vec3 p, vec2 c ) {
    c = normalize(c);
    float q = length(p.xy);
    return dot(c,vec2(q,p.z));
}

float cylinderSDF(vec3 p, float h, float r) {
    float inOutRadius = length(p.xy) - r;
    float inOutHeight = abs(p.z) - h/2.0;
    float insideDistance = min(max(inOutRadius, inOutHeight), 0.0);
    float outsideDistance = length(max(vec2(inOutRadius, inOutHeight), 0.0));
    return insideDistance + outsideDistance;
}

float sceneSDF(vec3 samplePoint, int env, out int id) {    
	
	obj[0].color = vec3(1.0, 1.0, 1.0);
	obj[0].prop = vec4(0.0, 1.0, 1.0, 0.0); // air
	
	obj[1].color = vec3(1.0, 0.9, 0.4);
	obj[1].prop = vec4(0.0, 0.0, 1.0, 10.0);

	obj[2].color = vec3(0.9, 0.9, 0.9);
	obj[2].prop = vec4(0.0, 0.0, 1.0, 10.0);

	obj[3].color = vec3(0.0, 0.0, 1.0);
	obj[3].prop = vec4(0.4, 0.4, 1.0, 10.0);
	
	obj[4].color = vec3(0.0, 1.0, 0.0);
	obj[4].prop = vec4(0.0, 0.0, 1.0, 10.0);
	
	obj[5].color = vec3(1.0, 1.0, 1.0);
	obj[5].prop = vec4(0.0, 0.85, 0.75, 10.0);
	
	obj[6].color = vec3(2.0, 0.0, 0.0);
	obj[6].prop = vec4(0.0, 0.7, 1.0, 10.0);
	
	obj[7].color = vec3(1.0, 0.0, 0.0);
	obj[7].prop = vec4(0.0, 0.7, 1.0, 10.0);
	
	obj[8].color = vec3(0.7, 0.7, 0.7);
	obj[8].prop = vec4(0.85, 0.0, 1.0, 10.0);
	
	float p[NUMBER_OF_OBJECTS + 1];
	
	p[0] = MAX_DIST; // air
	
	p[1] = intersectSDF(samplePoint.y, sphereSDF(samplePoint, 20.0));
	
	p[2] = intersectSDF(
		coneSDF(rotateX(radians(90.0)) * (samplePoint - vec3(0.0, 2.5, 0.0)),vec2(1.0, 0.5)),
		boxSDF(samplePoint - vec3(0.0, 2.5, 0.0), vec3(5.0, 5.0, 5.0))
	);
	p[2] = unionSDF(p[2], boxSDF(samplePoint - vec3(0.0, 2.5, 0.0), vec3(1.0, 5.0, 1.0)));
	p[2] = unionSDF(p[2], boxSDF(rotateY(radians(45.0))*samplePoint - vec3(0.0, 2.5, 0.0), vec3(1.0, 5.0, 1.0)));
	
	p[3] = boxSDF(samplePoint - vec3(0.0, 6.0, 0.0), vec3(1.0, 1.0, 1.0));
	
	p[4] = intersectSDF(p[3], coneSDF(rotateX(radians(-90.0)) * (samplePoint - vec3(0.0, 6.5, 0.0)), vec2(1.0, 0.4)));
	p[3] = differenceSDF(p[3], p[4]);
	
	p[5] = sphereSDF(samplePoint - vec3(-3.0, 2.0, 2.0), 1.0);
	
	p[6] = cylinderSDF(rotateX(radians(90.0)) * (samplePoint - vec3(3.0, 1.5, 2.0)), 3.0, 0.5);
	
	p[7] = cylinderSDF(rotateX(radians(90.0)) * (samplePoint - vec3(4.0, 1.0, 3.0)), 2.0, 0.35);
	
	p[8] = unionSDF(
		boxSDF(samplePoint - vec3(4.0, 6.0, 0.0), vec3(0.1, 3.0, 3.0)),
		boxSDF(samplePoint - vec3(-4.0, 6.0, 0.0), vec3(0.1, 3.0, 3.0))
	);
	
	if (env != 0) {
		p[0] = -p[env];
		p[env] = MAX_DIST;
	}
	
	id = 0;
	for(int i = 1; i <= NUMBER_OF_OBJECTS; ++i)
		if (p[id] >= p[i]) id = i;
	return p[id];
	
}

float distance(vec3 start, vec3 dir, float min_dist, float max_dist, int env, out int id) {

	float d, dist = min_dist;
    for (int i = 0; i < MAX_MARCHING_STEPS; ++i) {
		
        d = sceneSDF(start + dist*dir, env, id);
        if (d < EPSILON) return dist;
		if (dist >= max_dist) return max_dist;
        dist += d;
        
    }
    return max_dist;
	
}

float3 estimateNormal(float3 p, int env) {
	
	int id;
	float3 z1 = p + float3(EPSILON, 0, 0);
	float3 z2 = p - float3(EPSILON, 0, 0);
	float3 z3 = p + float3(0, EPSILON, 0);
	float3 z4 = p - float3(0, EPSILON, 0);
	float3 z5 = p + float3(0, 0, EPSILON);
	float3 z6 = p - float3(0, 0, EPSILON);
	float dx = sceneSDF(z1, env, id) - sceneSDF(z2, env, id);
	float dy = sceneSDF(z3, env, id) - sceneSDF(z4, env, id);
	float dz = sceneSDF(z5, env, id) - sceneSDF(z6, env, id);
	return normalize(float3(dx, dy, dz));
	
}

vec3 blinnPhong(vec3 color, float alpha, vec3 light, vec3 lightColor, float intensity, vec3 p, vec3 N, vec3 eye) {

    vec3 L = normalize(light - p);
    vec3 V = normalize(eye - p);
	vec3 H = normalize(L + V);
    
    float dotLN = dot(L, N);
    float dotNH = dot(N, H);
    
    return intensity * (color + lightColor * pow(max(dotNH,0.0), alpha)) * max(dotLN,0.0);
}

float shadowRay(vec3 p, vec3 light, int env, float k) {
	
	vec3 lDir = light - p;
	float dist = MIN_DIST, d, t, f = 1.0, s = 1.0;
	float start = dist, end = length(lDir);
	
	lDir = normalize(lDir);
	int id;
    
	while (true) {
		
		d = sceneSDF(p + dist*lDir, env, id);
        if (d < EPSILON) {
			f *= obj[id].prop.y;
			if (f < EPSILON) return 0.0;
			dist += MIN_DIST;
			start = dist;
			env = id;
			s = 1.0; 
		}
		if (dist >= end) return f*s;
		if (b_shadow)
			if ((d < end - dist) && (d < dist - start)) {
				t = k * d / dist;
				s = min(s, pow(1.0 / t, obj[id].prop.y) * t);
			}
		dist += d;
	
	}
	
}

float ambientOcclusion(vec3 p, vec3 normal, int env, float stepSize, int steps)
{
	float k, r = 0.0, t = 0.0;
	int id;
  
    for(int i = 0; i < steps; i++)
    {
        t += stepSize;
        k = (1.0 / pow(2.0, t)) * (t - sceneSDF(p + t * normal, env, id));
		if (obj[id].prop.y < EPSILON) r += k;
    }
    return max(0.0, 1.0 - 0.5 * r);
}

vec3 illumination(vec3 color, float alpha, vec3 p, vec3 normal, vec3 eye, int env) {
    
	vec3 colorPh = 0.4 * color; // ambientLight
	
	float lightIntensity = 10.0;
	vec3 lightColor = vec3(1.0, 1.0, 1.0);
	vec3 lightPos = vec3(2.5*sin(iTime), 7.0, 2.5*cos(iTime));
	
	float l = length(lightPos - p);
	lightIntensity /= l*l;
	
    colorPh += blinnPhong(color, alpha, lightPos, lightColor, lightIntensity, p, normal, eye);
	
	float shadow = shadowRay(p, lightPos, env, 30.0);
	shadow = 1.0 - (1.0 - shadow) * lightIntensity;
	
	float ambient = ambientOcclusion(p, normal, env, 0.1, 5);
		
	return colorPh * min(shadow, ambient);
}

vec3 fog(vec3 color, float d, vec3 fogColor) {
	
	float extintion = 0.02, inscattering = 0.01;
    return (color * exp(-d * extintion)) + (fogColor * (1.0 - exp(-d * inscattering)));
	
}

vec3 rayDirection(vec2 fragCoord, vec2 size, float fieldOfView) {
	
	float3 dir;
    dir.xy = fragCoord - size / 2.0;
    dir.z = -size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(dir);
	
}

void main() {
	
	setMode();
	vec3 color = vec3(0.0, 0.0, 0.0);
	
	rayInfo currentRay, ray1, ray2;
	rayInfo buf[RAY_BUF_SIZE];
	
	currentRay.p = g_camPos;
	currentRay.dir = normalize( float3x3(g_rayMatrix) * rayDirection(fragCoord, iResolution.xy, fieldOfView) );
	currentRay.d = 1.0;
	currentRay.env = 0;
	
	int id, n = 0;
	float3 hit_point, normal;
	float dist;
	
	vec3 fogColor = vec3(0.8, 0.8, 0.9);
	
	for(int j = 0; j < MAX_RAYS; ++j) {
		
		if (currentRay.d < EPSILON) {
			if (n == 0) break;
			currentRay = buf[--n];
		}
		
		dist = distance(currentRay.p, currentRay.dir, MIN_DIST, MAX_DIST, currentRay.env, id);

		if (dist > MAX_DIST - EPSILON) {
			color += (b_fog) ? fog(texture(skybox, currentRay.dir).xyz, dist, fogColor)* currentRay.d : 
				texture(skybox, currentRay.dir).xyz;
			currentRay.d = 0.0;
			continue;
		}
		hit_point = currentRay.p + dist * currentRay.dir;
		normal = estimateNormal(hit_point, currentRay.env);
		
		ray1.d = currentRay.d * obj[id].prop.x;
		ray2.d = currentRay.d * obj[id].prop.y;
		
		if (ray1.d > EPSILON) {
			currentRay.d -= ray1.d;
			ray1.p = hit_point;
			ray1.dir = normalize( reflect(currentRay.dir, normal) );
			ray1.env = currentRay.env;
		}
		if (ray2.d > EPSILON) {
			currentRay.d -= ray2.d;
			ray2.p = hit_point;
			ray2.dir = normalize( refract(currentRay.dir, normal, obj[id].prop.z / obj[currentRay.env].prop.z) );
			ray2.env = id;
		}
		if (currentRay.d > EPSILON) {
			vec3 light = obj[id].color;
//			if (id == 1)
//				if (mod(hit_point.x, 2.0) < mod(hit_point.z, 2.0))
//					light = vec3(1.0, 0.0, 0.0);
			light = illumination(light, obj[id].prop.w, hit_point, normal, currentRay.p, currentRay.env);
			if (b_fog) light = fog(light, dist, fogColor);
			color += currentRay.d * light;
		}
		if (ray1.d < ray2.d) {
			currentRay = ray2;
			ray2 = ray1;
		} else
			currentRay = ray1;
		
		if ((ray2.d > EPSILON) && (n < RAY_BUF_SIZE))
			buf[n++] = ray2;
		
	}
	fragColor = vec4(color, 1.0);
	return;
	
}