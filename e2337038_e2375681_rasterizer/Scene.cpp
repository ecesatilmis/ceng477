#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Mesh.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;


/*
	Transformations, clipping, culling, rasterization are done here.
	You may define helper functions.
*/

Matrix4 M_Translation(Translation *translation){

    double result[4][4] = {
            {1.0, 0.0, 0.0, translation->tx},
            {0.0, 1.0, 0.0, translation->ty},
            {0.0, 0.0, 1.0, translation->tz},
            {0.0, 0.0, 0.0, 1.0}
    };
    (Matrix4(result));
    return result;
}

Matrix4 M_Scaling(Scaling *scaling){

    double result[4][4] = {
            {scaling->sx, 0.0, 0.0, 0.0},
            {0.0, scaling->sy, 0.0, 0.0},
            {0.0, 0.0, scaling->sz, 0.0},
            {0.0, 0.0, 0.0, 1.0}
    };
    (Matrix4(result));
    return result;
}

Matrix4 M_Rotation(Rotation *rotation){
    Vec3 u, v, w;
    u.x = rotation->ux;
    u.y = rotation->uy;
    u.z = rotation->uz;
    u = normalizeVec3(u);

    if (std::abs(u.x) <= std::abs(u.y) && std::abs(u.x) <= std::abs(u.z)){
        v.x = 0;
        v.y = u.z;
        v.z = -u.y;
    }
    else if (std::abs(u.y) <= std::abs(u.x) && std::abs(u.y) <= std::abs(u.z)){
        v.x = u.z;
        v.y = 0;
        v.z = -u.x;
    }
    else if (std::abs(u.z) <= std::abs(u.y) &&std::abs(u.z) <= std::abs(u.x)){
        v.x = u.y;
        v.y = -u.x;
        v.z = 0;
    }
    v = normalizeVec3(v);
    w = crossProductVec3(u, v);
    w = normalizeVec3(w);
    double M_matrix[4][4] = {{u.x, u.y, u.z, 0},
                             {v.x, v.y, v.z, 0},
                             {w.x, w.y, w.z, 0},
                             {0,          0,          0,          1}};

    double M_inverse[4][4] = {{u.x, v.x, w.x, 0},
                              {u.y, v.y, w.y, 0},
                              {u.z, v.z, w.z, 0},
                              {0,          0,          0,          1}};
    double theta = (rotation->angle*M_PI)/180.0;
    double Rx_theta[4][4] = {{1, 0,          0,           0},
                             {0, cos(theta), -sin(theta), 0},
                             {0, sin(theta), cos(theta),  0},
                             {0, 0,          0,           1}};


    Matrix4 Rx_times_M = multiplyMatrixWithMatrix(Matrix4(Rx_theta), Matrix4(M_matrix));
    Matrix4 result = multiplyMatrixWithMatrix(Matrix4(M_inverse), Rx_times_M);

    return result;
}

Matrix4 M_Transform(Mesh* mesh, Scene *scene){
    Matrix4 result = getIdentityMatrix();

    for (int i = 0; i < mesh->numberOfTransformations; i++){
        int transformation_id = mesh->transformationIds[i];
        if (mesh->transformationTypes[i] == 's') {
            Scaling *scaling;
            scaling = scene->scalings[transformation_id - 1];
            Matrix4 s = M_Scaling(scaling);
            result = multiplyMatrixWithMatrix(s, result);
        }
        else if (mesh->transformationTypes[i] == 't') {
            Translation *translation;
            translation = scene->translations[transformation_id - 1];
            Matrix4 t = M_Translation(translation);
            result = multiplyMatrixWithMatrix(t, result);
        }
        else {
            Rotation *rotation;
            rotation = scene->rotations[transformation_id - 1];
            Matrix4 r = M_Rotation(rotation);
            result = multiplyMatrixWithMatrix(r,result);
        }
    }
    return result;
}

Matrix4 M_Camera(Camera *camera){

    double result[4][4] = {
            {camera->u.x, camera->u.y, camera->u.z, -(camera->u.x * camera->pos.x + camera->u.y * camera->pos.y + camera->u.z * camera->pos.z)},
            {camera->v.x, camera->v.y, camera->v.z, -(camera->v.x * camera->pos.x + camera->v.y * camera->pos.y + camera->v.z * camera->pos.z)},
            {camera->w.x, camera->w.y, camera->w.z, -(camera->w.x * camera->pos.x + camera->w.y * camera->pos.y + camera->w.z * camera->pos.z)},
            {0, 0, 0, 1}
    };
    (Matrix4(result));
    return result;

}

Matrix4 M_Projection(Camera *camera){
    Matrix4 final_result;
    if(camera->projectionType){
        double result[4][4] = {
                {(2 * camera->near) / (camera->right - camera->left), 0, (camera->right + camera->left) / (camera->right - camera->left), 0},
                {0, (2 * camera->near) / (camera->top - camera->bottom), (camera->top + camera->bottom) / (camera->top - camera->bottom), 0},
                {0, 0, -(camera->far + camera->near) / (camera->far - camera->near), -(2 * camera->far * camera->near) / (camera->far - camera->near)},
                {0, 0, -1, 0}
        };
        final_result = Matrix4(result);
    }
    else{
        double result[4][4] = {
                {2.0 / (camera->right - camera->left), 0,0,-((camera->right + camera->left)/(camera->right - camera->left))},
                {0, 2.0 / (camera->top - camera->bottom), 0,-((camera->top + camera->bottom) / (camera->top - camera->bottom))},
                {0,0, -(2 / (camera->far - camera->near)), -(( camera->far + camera->near) / (camera->far - camera->near))},
                {0,0,0,1}
        };

        final_result = Matrix4(result);
    }

    return final_result;
}

Matrix4 M_Viewport(Camera *camera){

    double result[4][4] = {{(double) camera->horRes / 2.0,0,0,(double) (camera->horRes -1) / 2.0},
                           {0,(double) camera->verRes / 2.0,0,(double) (camera->verRes - 1) / 2.0},
                           {0, 0, 0.5,0.5},
                           {0, 0, 0, 1}};
    (Matrix4(result));
    return result;
}

struct Line{
    Vec4 begin;
    Vec4 end;
    bool visible;
};

int calculate_color(int colorId1, int colorId2, double r_old, double r_new, Scene *scene){
    Color *result = new Color();
    Color tmp1 = *scene->colorsOfVertices[colorId1-1];
    Color tmp2 = *scene->colorsOfVertices[colorId2-1];
    result->r = floor(tmp1.r * (r_old - r_new) + tmp2.r * r_new);
    result->g = floor(tmp1.g * (r_old - r_new) + tmp2.g * r_new);
    result->b = floor(tmp1.b * (r_old - r_new) + tmp2.b * r_new);
    scene->colorsOfVertices.push_back(result);
    return scene->colorsOfVertices.size();
}


bool visible(double distance, double difference, double *te, double *tl){
    if(distance > 0){
        double t = difference / distance;
        if (t > *tl)
            return false;
        if (t > *te)
            *te = t;
    }
    else if (distance < 0){
        double t = difference / distance;
        if (t < *te)
            return false;
        if (t < *tl)
            *tl = t;
    }
    else if (difference > 0){
        return false;
    }
    return true;
}

void clipping(Vec4* begin, Vec4* end, Scene* scene, vector<Line>* result){
    double te = 0, tl = 1;
    double x_distance = end->x-begin->x;
    double y_distance = end->y-begin->y;
    double z_distance = end->z-begin->z;
    if(visible(x_distance, -1 - begin->x, &te, &tl)){ // left
        if(visible(-x_distance, begin->x - 1, &te, &tl)){ // right
            if(visible(y_distance, -1 - begin->y, &te, &tl)){ // bottom
                if(visible(-y_distance, begin->y - 1, &te, &tl)){ // top
                    if(visible(z_distance, -1 - begin->z, &te, &tl)){ // front
                        if(visible(-z_distance, begin->z - 1, &te, &tl)){ // back
                            double ratio_before = sqrt(x_distance * x_distance + y_distance * y_distance + z_distance * z_distance);
                            Line resultant_line; resultant_line.visible = true; resultant_line.begin = *begin; resultant_line.end = *end;
                            if(tl<1){
                                resultant_line.end.x = begin->x + x_distance * tl;
                                resultant_line.end.y = begin->y + y_distance * tl;
                                resultant_line.end.z = begin->z + z_distance * tl;
                                resultant_line.end.t = 1;
                                double ratio_after = sqrt((begin->x - resultant_line.end.x) * (begin->x - resultant_line.end.x) +
                                        (begin->y - resultant_line.end.y) * (begin->y - resultant_line.end.y) +
                                        (begin->z - resultant_line.end.z) * (begin->z - resultant_line.end.z));
                                resultant_line.end.colorId = calculate_color(begin->colorId, end->colorId, ratio_before, ratio_after, scene);
                            }
                            if(te>0){
                                resultant_line.begin.x = begin->x + x_distance * te;
                                resultant_line.begin.y = begin->y + y_distance * te;
                                resultant_line.begin.z = begin->z + z_distance * te;
                                resultant_line.begin.t = 1;
                                double ratio_after = sqrt((begin->x - resultant_line.begin.x) * (begin->x - resultant_line.begin.x) +
                                                          (begin->y - resultant_line.begin.y) * (begin->y - resultant_line.begin.y) +
                                                          (begin->z - resultant_line.begin.z) * (begin->z - resultant_line.begin.z));
                                resultant_line.begin.colorId = calculate_color(begin->colorId, end->colorId, ratio_before, ratio_after, scene);
                            }
                            result->push_back(resultant_line);
                        }
                    }
                }
            }
        }
    }
}

bool backface_culling(Vec3 ver1, Vec3 ver2, Vec3 ver3)
{
    Vec3 edge12 = subtractVec3(ver2, ver1);
    Vec3 edge13 = subtractVec3(ver3, ver1);
    Vec3 normal = normalizeVec3(crossProductVec3(edge12, edge13));
    double angle = dotProductVec3(normal, ver1);
    return (angle < 0);
}

void generate_line(vector<Vec4> &rearrangedVertices, Mesh* mesh, Scene *scene, Vec3 gaze, bool culling, vector<Line> *result){
    for(Triangle &triangle : mesh->triangles){

        Vec4 v1 = rearrangedVertices[triangle.getFirstVertexId()-1];
        Vec4 v2 = rearrangedVertices[triangle.getSecondVertexId()-1];
        Vec4 v3 = rearrangedVertices[triangle.getThirdVertexId()-1];

        Vec3 vertex1(v1.x, v1.y, v1.z, -1);
        Vec3 vertex2(v2.x, v2.y, v2.z, -1);
        Vec3 vertex3(v3.x, v3.y, v3.z, -1);

        if(!backface_culling(vertex1, vertex2, vertex3) || !culling){
            clipping(&v1, &v2, scene, result);
            clipping(&v2, &v3, scene, result);
            clipping(&v3, &v1, scene, result);
        }
    }
}

void draw_line(Line line, Camera* camera, Scene* scene){
    double slope = (line.end.y - line.begin.y) / (line.end.x - line.begin.x);
    double d;
    int x, y;
    Color current, begin, end;
    if (slope >= 0 && abs(slope) < 1){
        if (line.begin.x < line.end.x){
            y = line.begin.y + 0.5;
            d = 2 * (line.begin.y - line.end.y) + (line.end.x - line.begin.x);
            begin = *scene->colorsOfVertices[line.begin.colorId - 1];
            end = *scene->colorsOfVertices[line.end.colorId - 1];
            current = begin;
            Color diff((end - begin) / (line.end.x - line.begin.x));
            for (x = line.begin.x + 0.5; x < line.end.x; x++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        y++;
                        d += 2 * (line.begin.y - line.end.y + line.end.x - line.begin.x);
                    }else{
                        d += 2 * (line.begin.y - line.end.y);
                    }
                    current += diff;
                }
            }
        }else{
            y = line.end.y + 0.5;
            d = 2 * (line.end.y - line.begin.y) + (line.begin.x - line.end.x);
            begin = *scene->colorsOfVertices[line.end.colorId - 1];
            end = *scene->colorsOfVertices[line.begin.colorId - 1];
            current = begin;
            Color diff((end - begin) / (line.begin.x - line.end.x));
            for (x = line.end.x + 0.5; x < line.begin.x; x++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        y++;
                        d += 2 * (line.end.y - line.begin.y + line.begin.x - line.end.x);
                    }else{
                        d += 2 * (line.end.y - line.begin.y);
                    }
                    current += diff;
                }
            }
        }
    }else if (slope >= 0 && abs(slope) >= 1){
        if (line.begin.y < line.end.y){
            x = line.begin.x + 0.5;
            d = (line.end.y - line.begin.y) + 2 * (line.begin.x - line.end.x);
            begin = *scene->colorsOfVertices[line.begin.colorId - 1];
            end = *scene->colorsOfVertices[line.end.colorId - 1];
            current = begin;
            Color diff((end - begin) / (line.end.y - line.begin.y));
            for (y = line.begin.y + 0.5; y < line.end.y; y++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        x++;
                        d += 2 * (line.end.y - line.begin.y + line.begin.x - line.end.x);
                    }else{
                        d += 2 * (line.begin.x - line.end.x);
                    }
                    current += diff;
                }
            }
        }else{
            x = line.end.x + 0.5;
            d = (line.begin.y - line.end.y) + 2 * (line.end.x - line.begin.x);
            begin = *scene->colorsOfVertices[line.end.colorId - 1];
            end = *scene->colorsOfVertices[line.begin.colorId - 1];
            current = begin;
            Color diff((end - begin) / (line.begin.y - line.end.y));
            for (y = line.end.y + 0.5; y < line.begin.y; y++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        x++;
                        d += 2 * (line.begin.y - line.end.y + line.end.x - line.begin.x);
                    }else{
                        d += 2 * (line.end.x - line.begin.x);
                    }
                    current += diff;
                }
            }
        }
    }else if (slope < 0 && abs(slope) < 1){
        if (line.begin.x > line.end.x){
            y = line.begin.y + 0.5;
            d = 2 * (line.begin.y - line.end.y) + (line.begin.x - line.end.x);
            begin = *scene->colorsOfVertices[line.begin.colorId - 1];
            end = *scene->colorsOfVertices[line.end.colorId - 1];
            current = begin;
            Color diff((begin - end) / (line.begin.x - line.end.x));
            for (x = line.begin.x + 0.5; x > line.end.x; x--){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        y++;
                        d += 2 * (line.begin.y - line.end.y + line.begin.x - line.end.x);
                    }else{
                        d += 2 * (line.begin.y - line.end.y);
                    }
                    current -= diff;
                }
            }
        }else{
            y = line.end.y + 0.5;
            d = 2 * (line.end.y - line.begin.y) + (line.end.x - line.begin.x);
            begin = *scene->colorsOfVertices[line.end.colorId - 1];
            end = *scene->colorsOfVertices[line.begin.colorId - 1];
            current = begin;
            Color diff((begin - end) / (line.end.x - line.begin.x));
            for (x = line.end.x + 0.5; x > line.begin.x; x--){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        y++;
                        d += 2 * (line.end.y - line.begin.y + line.end.x - line.begin.x);
                    }else{
                        d += 2 * (line.end.y - line.begin.y);
                    }
                    current -= diff;
                }
            }
        }
    }else if (slope < 0 && abs(slope) >= 1){
        if (line.begin.y < line.end.y){
            x = line.begin.x + 0.5;
            d = (line.end.y - line.begin.y) + 2 * (line.end.x - line.begin.x);
            begin = *scene->colorsOfVertices[line.begin.colorId - 1];
            end = *scene->colorsOfVertices[line.end.colorId - 1];
            current = begin;
            Color diff((begin - end) / (line.end.y - line.begin.y));
            for (y = line.begin.y + 0.5; y < line.end.y; y++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        x--;
                        d += 2 * (line.end.y - line.begin.y + line.end.x - line.begin.x);
                    }else{
                        d += 2 * (line.end.x - line.begin.x);
                    }
                    current -= diff;
                }
            }
        }else{
            x = line.end.x + 0.5;
            d = (line.begin.y - line.end.y) + 2 * (line.begin.x - line.end.x);
            begin = *scene->colorsOfVertices[line.end.colorId - 1];
            end = *scene->colorsOfVertices[line.begin.colorId - 1];
            current = begin;
            Color diff((begin - end) / (line.begin.y - line.end.y));
            for (y = line.end.y + 0.5; y < line.begin.y; y++){
                if (0 <= x && x < camera->horRes && 0 <= y && y < camera->verRes){
                    scene->image[x][y] = current;
                    if (d < 0){
                        x--;
                        d += 2 * (line.begin.y - line.end.y + line.begin.x - line.end.x);
                    }else{
                        d += 2 * (line.begin.x - line.end.x);
                    }
                    current -= diff;
                }
            }
        }
    }
}


double Scene::f(double x, double y, Vec4 &v1, Vec4 &v2){
    return x*(v1.y-v2.y) + y*(v2.x-v1.x) + v1.x*v2.y - v1.y*v2.x;
}

void Scene::T_Rasterization(Scene *scene,Camera *camera, Vec4 &v0, Vec4 &v1, Vec4 &v2, bool culling){
	Vec3 ver1(v0.x, v0.y, v0.z, -1), ver2(v1.x, v1.y, v1.z, -1), ver3(v2.x, v2.y, v2.z, -1);

	if (!backface_culling(ver1, ver2, ver3) || !culling){
	    int x_min, y_min, x_max, y_max;
	    x_min = (std::max(0.0, std::min(std::min(v1.x, v2.x), v0.x)));
	    y_min = (std::max(0.0, std::min(std::min(v1.y, v2.y), v0.y)));
	    x_max = (std::max(std::max(v1.x, v2.x), v0.x));
	    y_max = (std::max(std::max(v1.y, v2.y), v0.y));
	    x_min = x_min <= camera->horRes - 1 ? x_min : camera->horRes - 1;
	    y_min = y_min <= camera->verRes - 1 ? y_min : camera->verRes - 1;
	    x_max = x_max >= camera->horRes - 1 ? camera->horRes - 1 : x_max;
	    y_max = y_max >= camera->verRes - 1 ? camera->verRes - 1 : y_max;

	    double f_12 = f(v0.x, v0.y, v1, v2);
	    double f_20 = f(v1.x, v1.y, v2, v0);
	    double f_01 = f(v2.x, v2.y, v0, v1);

	    double alpha, beta, gamma;

	    Color v0_c = *(scene->colorsOfVertices[v0.colorId - 1]);
	    Color v1_c = *(scene->colorsOfVertices[v1.colorId - 1]);
	    Color v2_c = *(scene->colorsOfVertices[v2.colorId - 1]);

	    for (int y = y_min; y <= y_max; y++) {
	        for (int x = x_min; x <= x_max; x++) {

	            alpha = (f(x, y, v1, v2))/ f_12;
	            beta = f(x, y, v2, v0) / f_20;
	            gamma = f(x, y, v0, v1) / f_01;

	            if (alpha >= 0 && beta >= 0 && gamma >= 0){

	                scene->image[x][y].r = alpha * v0_c.r + beta * v1_c.r + gamma * v2_c.r;
	                scene->image[x][y].g = alpha * v0_c.g + beta * v1_c.g + gamma * v2_c.g;
	                scene->image[x][y].b = alpha * v0_c.b + beta * v1_c.b + gamma * v2_c.b;

	            }
	        }
	    }
	}
}

void Scene::forwardRenderingPipeline(Camera *camera)
{
    Matrix4 camera_t = M_Camera(camera);

    Matrix4 projection = M_Projection(camera);
    Matrix4 projection_times_camera = multiplyMatrixWithMatrix(projection, camera_t);
    Matrix4 viewport = M_Viewport(camera);

    for(Mesh *mesh : meshes){
        Matrix4 modelling = M_Transform(mesh, this);
        Matrix4 p_times_c_times_m = multiplyMatrixWithMatrix(projection_times_camera, modelling);
        vector<Vec4> rearranged_vertices;

        for(Vec3 *vertex : vertices){
            Vec4 vec(vertex->x, vertex->y, vertex->z, 1.0, vertex->colorId);
            vec = multiplyMatrixWithVec4(p_times_c_times_m,vec);
            vec.x = vec.x / vec.t; vec.y = vec.y / vec.t; vec.z = vec.z / vec.t; vec.t = vec.t / vec.t;
            rearranged_vertices.push_back(vec);
        }

        if(mesh->type){
            for(int j=0;j<rearranged_vertices.size();j++)
                rearranged_vertices[j] = multiplyMatrixWithVec4(viewport, rearranged_vertices[j]);
            for(Triangle &triangle : mesh->triangles){
                T_Rasterization(this, camera,
                                rearranged_vertices[triangle.getFirstVertexId() - 1],
                                rearranged_vertices[triangle.getSecondVertexId() - 1],
                                rearranged_vertices[triangle.getThirdVertexId() - 1], cullingEnabled);
            }
        }
        else{
            vector<Line> lines;

            generate_line(rearranged_vertices, mesh, this, camera->gaze, cullingEnabled, &lines);

            rearranged_vertices.clear();
            for(Line &line : lines){
                line.begin = multiplyMatrixWithVec4(viewport, line.begin);
                line.end = multiplyMatrixWithVec4(viewport, line.end);
            }
            for(Line &line : lines){
                if(line.visible){
                    draw_line(line, camera, this);
                }
            }
            lines.clear();
        }
    }
}


/*
	Parses XML file
*/

Scene::Scene(const char *xmlPath)
{
    const char *str;
    XMLDocument xmlDoc;
    XMLElement *pElement;

    xmlDoc.LoadFile(xmlPath);

    XMLNode *pRoot = xmlDoc.FirstChild();

    // read background color
    pElement = pRoot->FirstChildElement("BackgroundColor");
    str = pElement->GetText();
    sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

    // read culling
    pElement = pRoot->FirstChildElement("Culling");
    if (pElement != NULL) {
        str = pElement->GetText();

        if (strcmp(str, "enabled") == 0) {
            cullingEnabled = true;
        }
        else {
            cullingEnabled = false;
        }
    }

    // read cameras
    pElement = pRoot->FirstChildElement("Cameras");
    XMLElement *pCamera = pElement->FirstChildElement("Camera");
    XMLElement *camElement;
    while (pCamera != NULL)
    {
        Camera *cam = new Camera();

        pCamera->QueryIntAttribute("id", &cam->cameraId);

        // read projection type
        str = pCamera->Attribute("type");

        if (strcmp(str, "orthographic") == 0) {
            cam->projectionType = 0;
        }
        else {
            cam->projectionType = 1;
        }

        camElement = pCamera->FirstChildElement("Position");
        str = camElement->GetText();
        sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

        camElement = pCamera->FirstChildElement("Gaze");
        str = camElement->GetText();
        sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

        camElement = pCamera->FirstChildElement("Up");
        str = camElement->GetText();
        sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

        cam->gaze = normalizeVec3(cam->gaze);
        cam->u = crossProductVec3(cam->gaze, cam->v);
        cam->u = normalizeVec3(cam->u);

        cam->w = inverseVec3(cam->gaze);
        cam->v = crossProductVec3(cam->u, cam->gaze);
        cam->v = normalizeVec3(cam->v);

        camElement = pCamera->FirstChildElement("ImagePlane");
        str = camElement->GetText();
        sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
               &cam->left, &cam->right, &cam->bottom, &cam->top,
               &cam->near, &cam->far, &cam->horRes, &cam->verRes);

        camElement = pCamera->FirstChildElement("OutputName");
        str = camElement->GetText();
        cam->outputFileName = string(str);

        cameras.push_back(cam);

        pCamera = pCamera->NextSiblingElement("Camera");
    }

    // read vertices
    pElement = pRoot->FirstChildElement("Vertices");
    XMLElement *pVertex = pElement->FirstChildElement("Vertex");
    int vertexId = 1;

    while (pVertex != NULL)
    {
        Vec3 *vertex = new Vec3();
        Color *color = new Color();

        vertex->colorId = vertexId;

        str = pVertex->Attribute("position");
        sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

        str = pVertex->Attribute("color");
        sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

        vertices.push_back(vertex);
        colorsOfVertices.push_back(color);

        pVertex = pVertex->NextSiblingElement("Vertex");

        vertexId++;
    }

    // read translations
    pElement = pRoot->FirstChildElement("Translations");
    XMLElement *pTranslation = pElement->FirstChildElement("Translation");
    while (pTranslation != NULL)
    {
        Translation *translation = new Translation();

        pTranslation->QueryIntAttribute("id", &translation->translationId);

        str = pTranslation->Attribute("value");
        sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

        translations.push_back(translation);

        pTranslation = pTranslation->NextSiblingElement("Translation");
    }

    // read scalings
    pElement = pRoot->FirstChildElement("Scalings");
    XMLElement *pScaling = pElement->FirstChildElement("Scaling");
    while (pScaling != NULL)
    {
        Scaling *scaling = new Scaling();

        pScaling->QueryIntAttribute("id", &scaling->scalingId);
        str = pScaling->Attribute("value");
        sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

        scalings.push_back(scaling);

        pScaling = pScaling->NextSiblingElement("Scaling");
    }

    // read rotations
    pElement = pRoot->FirstChildElement("Rotations");
    XMLElement *pRotation = pElement->FirstChildElement("Rotation");
    while (pRotation != NULL)
    {
        Rotation *rotation = new Rotation();

        pRotation->QueryIntAttribute("id", &rotation->rotationId);
        str = pRotation->Attribute("value");
        sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

        rotations.push_back(rotation);

        pRotation = pRotation->NextSiblingElement("Rotation");
    }

    // read meshes
    pElement = pRoot->FirstChildElement("Meshes");

    XMLElement *pMesh = pElement->FirstChildElement("Mesh");
    XMLElement *meshElement;
    while (pMesh != NULL)
    {
        Mesh *mesh = new Mesh();

        pMesh->QueryIntAttribute("id", &mesh->meshId);

        // read projection type
        str = pMesh->Attribute("type");

        if (strcmp(str, "wireframe") == 0) {
            mesh->type = 0;
        }
        else {
            mesh->type = 1;
        }

        // read mesh transformations
        XMLElement *pTransformations = pMesh->FirstChildElement("Transformations");
        XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

        while (pTransformation != NULL)
        {
            char transformationType;
            int transformationId;

            str = pTransformation->GetText();
            sscanf(str, "%c %d", &transformationType, &transformationId);

            mesh->transformationTypes.push_back(transformationType);
            mesh->transformationIds.push_back(transformationId);

            pTransformation = pTransformation->NextSiblingElement("Transformation");
        }

        mesh->numberOfTransformations = mesh->transformationIds.size();

        // read mesh faces
        char *row;
        char *clone_str;
        int v1, v2, v3;
        XMLElement *pFaces = pMesh->FirstChildElement("Faces");
        str = pFaces->GetText();
        clone_str = strdup(str);

        row = strtok(clone_str, "\n");
        while (row != NULL)
        {
            int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

            if (result != EOF) {
                mesh->triangles.push_back(Triangle(v1, v2, v3));
            }
            row = strtok(NULL, "\n");
        }
        mesh->numberOfTriangles = mesh->triangles.size();
        meshes.push_back(mesh);

        pMesh = pMesh->NextSiblingElement("Mesh");
    }
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
    if (this->image.empty())
    {
        for (int i = 0; i < camera->horRes; i++)
        {
            vector<Color> rowOfColors;

            for (int j = 0; j < camera->verRes; j++)
            {
                rowOfColors.push_back(this->backgroundColor);
            }

            this->image.push_back(rowOfColors);
        }
    }
    else
    {
        for (int i = 0; i < camera->horRes; i++)
        {
            for (int j = 0; j < camera->verRes; j++)
            {
                this->image[i][j].r = this->backgroundColor.r;
                this->image[i][j].g = this->backgroundColor.g;
                this->image[i][j].b = this->backgroundColor.b;
            }
        }
    }
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
    if (value >= 255.0)
        return 255;
    if (value <= 0.0)
        return 0;
    return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
    ofstream fout;

    fout.open(camera->outputFileName.c_str());

    fout << "P3" << endl;
    fout << "# " << camera->outputFileName << endl;
    fout << camera->horRes << " " << camera->verRes << endl;
    fout << "255" << endl;

    for (int j = camera->verRes - 1; j >= 0; j--)
    {
        for (int i = 0; i < camera->horRes; i++)
        {
            fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
                 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
                 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
        }
        fout << endl;
    }
    fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
    string command;

    // call command on Ubuntu
    if (osType == 1)
    {
        command = "convert " + ppmFileName + " " + ppmFileName + ".png";
        system(command.c_str());
    }

        // call command on Windows
    else if (osType == 2)
    {
        command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
        system(command.c_str());
    }

        // default action - don't do conversion
    else
    {
    }
}