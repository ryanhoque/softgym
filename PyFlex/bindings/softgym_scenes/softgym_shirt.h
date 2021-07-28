#pragma once
#include <iostream>
#include <vector>
#include <cmath>

inline float DIST(Point3& a, Point3& b) {
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2) + pow(a.z - b.z,2));
}

class SoftgymShirt : public Scene
{
public:
    float cam_x;
    float cam_y;
    float cam_z;
    float cam_angle_x;
    float cam_angle_y;
    float cam_angle_z;
    int cam_width;
    int cam_height;

    SoftgymShirt(const char* name) : Scene(name) {}

    float get_param_float(py::array_t<float> scene_params, int idx)
    {
        auto ptr = (float *) scene_params.request().ptr;
        float out = ptr[idx];
        return out;
    }

    //params ordering: xpos, ypos, zpos, xsize, zsize, stretch, bend, shear
    // render_type, cam_X, cam_y, cam_z, angle_x, angle_y, angle_z, width, height
    void Initialize(py::array_t<float> scene_params, int thread_idx=0)
    {
        auto ptr = (float *) scene_params.request().ptr;
        float initX = ptr[0];
        float initY = ptr[1];
        float initZ = ptr[2];

        int dimx = (int)ptr[3];
        int dimz = (int)ptr[4];
        float radius = 0.00625f;

        int render_type = ptr[8]; // 0: only points, 1: only mesh, 2: points + mesh

        cam_x = ptr[9];
        cam_y = ptr[10];
        cam_z = ptr[11];
        cam_angle_x = ptr[12];
        cam_angle_y = ptr[13];
        cam_angle_z = ptr[14];
        cam_width = int(ptr[15]);
        cam_height = int(ptr[16]);

        // Cloth
        float stretchStiffness = ptr[5]; //0.9f;
        float bendStiffness = ptr[6]; //1.0f;
        float shearStiffness = ptr[7]; //0.9f;
        int phase = NvFlexMakePhase(0, eNvFlexPhaseSelfCollide | eNvFlexPhaseSelfCollideFilter);
        float mass = float(ptr[17])/6544;    // avg bath towel is 500-700g
        int flip_mesh = int(ptr[18]); // Flip half

        float stiffness = 0.08;
        float invMass = 1.0f/mass;
        Vec3 lower = Vec3(initX, -initY, initZ);
        Vec3 velocity = 0.0f;

        //CreateSpringGrid(Vec3(initX, -initY, initZ), dimx, dimz, 1, radius, phase, stretchStiffness, bendStiffness, shearStiffness, 0.0f, 1.0f/mass);
	Mesh* m = ImportMesh("/home/ryanhoque/softgym/PyFlex/data/tshirt-rotated.obj");
        if (!m) 
        {
            cout << "no mesh" << endl;
            return;
        }

        cout << "mesh faces:" << m->GetNumFaces() << endl;

        // rotate mesh? normalize?
        // m->Transform(RotationMatrix(rotation, Vec3(0.0f, 1.0f, 0.0f)));
        float avgEdgeLen = 0.;
        //
        for (uint32_t i=0; i < m->GetNumFaces(); ++i)
        {
            // create particles
            uint32_t a = m->m_indices[i*3+0];
            uint32_t b = m->m_indices[i*3+1];
            uint32_t c = m->m_indices[i*3+2];

            Point3& v0 = m->m_positions[a];
            Point3& v1 = m->m_positions[b];
            Point3& v2 = m->m_positions[c];

            avgEdgeLen += DIST(v0, v1) + DIST(v1, v2) + DIST(v2, v0);
        }
        avgEdgeLen /= 3 * m->GetNumFaces();
        float scale =  radius / avgEdgeLen * 2.5;

        cout<<"Scale:"<<scale<<endl;

        Vec3 meshLower, meshUpper;
        m->GetBounds(meshLower, meshUpper);

        Matrix44 xform = ScaleMatrix(scale)*TranslationMatrix(Point3(-meshLower));
        m->Transform(xform);

        m->GetBounds(meshLower, meshUpper);

        // index of particles
        uint32_t baseIndex = uint32_t(g_buffers->positions.size());
        uint32_t currentIndex = baseIndex;

        // maps vertex by position
        // maps position to particle index
        map<vector<float>, uint32_t> vertex;
        map<vector<float>, uint32_t>::iterator it;

        // maps from vertex index to particle index
        map<uint32_t, uint32_t> indMap;

        // to check for duplicate connections
        map<uint32_t,vector<uint32_t> > edgeMap;

        // loop through the faces
        for (uint32_t i=0; i < m->GetNumFaces(); ++i)
        {
            // create particles
            uint32_t a = m->m_indices[i*3+0];
            uint32_t b = m->m_indices[i*3+1];
            uint32_t c = m->m_indices[i*3+2];

            Point3& v0 = m->m_positions[a];
            Point3& v1 = m->m_positions[b];
            Point3& v2 = m->m_positions[c];

            //sortInd(&a, &b, &c);

            float arr0[] = {v0.x, v0.y, v0.z};
            float arr1[] = {v1.x, v1.y, v1.z};
            float arr2[] = {v2.x, v2.y, v2.z};
            vector<float> pos0(arr0, arr0 + sizeof(arr0)/sizeof(arr0[0]));
            vector<float> pos1(arr1, arr1 + sizeof(arr1)/sizeof(arr1[0]));
            vector<float> pos2(arr2, arr2 + sizeof(arr2)/sizeof(arr2[0]));

            it = vertex.find(pos0);
            if (it == vertex.end()) {
                vertex[pos0] = currentIndex;
                indMap[a] = currentIndex++;
                Vec3 p0 = lower + meshLower + Vec3(v0.x, v0.y, v0.z);
                g_buffers->positions.push_back(Vec4(p0.x, p0.y, p0.z, invMass));
                g_buffers->velocities.push_back(velocity);
                g_buffers->phases.push_back(phase);
            }
            else
            {
                indMap[a] = it->second;
            }

            it = vertex.find(pos1);
            if (it == vertex.end()) {
                vertex[pos1] = currentIndex;
                indMap[b] = currentIndex++;
                Vec3 p1 = lower + meshLower + Vec3(v1.x, v1.y, v1.z);
                g_buffers->positions.push_back(Vec4(p1.x, p1.y, p1.z, invMass));
                g_buffers->velocities.push_back(velocity);
                g_buffers->phases.push_back(phase);
            }
            else
            {
                indMap[b] = it->second;
            }

            it = vertex.find(pos2);
            if (it == vertex.end()) {
                vertex[pos2] = currentIndex;
                indMap[c] = currentIndex++;
                Vec3 p2 = lower + meshLower + Vec3(v2.x, v2.y, v2.z);
                g_buffers->positions.push_back(Vec4(p2.x, p2.y, p2.z, invMass));
                g_buffers->velocities.push_back(velocity);
                g_buffers->phases.push_back(phase);
            }
            else
            {
                indMap[c] = it->second;
            }

            // create triangles
            g_buffers->triangles.push_back(indMap[a]);
            g_buffers->triangles.push_back(indMap[b]);
            g_buffers->triangles.push_back(indMap[c]);

            // TODO: normals?

            // connect springs
            
            // add spring if not duplicate
            vector<uint32_t>::iterator it;
            // for a-b
            if (edgeMap.find(a) == edgeMap.end())
            {
                CreateSpring(indMap[a], indMap[b], stiffness);
        //                cout<<"Spring len:"<<DIST(indMap[a], indMap[b])
                edgeMap[a].push_back(b);
            }
            else
            {

                it = find(edgeMap[a].begin(), edgeMap[a].end(), b);
                if (it == edgeMap[a].end())
                {
                    CreateSpring(indMap[a], indMap[b], stiffness);
                    edgeMap[a].push_back(b);
                }
            }

            // for a-c
            if (edgeMap.find(a) == edgeMap.end())
            {
                CreateSpring(indMap[a], indMap[c], stiffness);
                edgeMap[a].push_back(c);
            }
            else
            {

                it = find(edgeMap[a].begin(), edgeMap[a].end(), c);
                if (it == edgeMap[a].end())
                {
                    CreateSpring(indMap[a], indMap[c], stiffness);
                    edgeMap[a].push_back(c);
                }
            }

            // for b-c
            if (edgeMap.find(b) == edgeMap.end())
            {
                CreateSpring(indMap[b], indMap[c], stiffness);
                edgeMap[b].push_back(c);
            }
            else
            {

                it = find(edgeMap[b].begin(), edgeMap[b].end(), c);
                if (it == edgeMap[b].end())
                {
                    CreateSpring(indMap[b], indMap[c], stiffness);
                    edgeMap[b].push_back(c);
                }
            }         
        }

        g_numSubsteps = 4;
        g_params.numIterations = 30;

        g_params.dynamicFriction = 0.75f;
        g_params.particleFriction = 1.0f;
        g_params.damping = 1.0f;
        g_params.sleepThreshold = 0.02f;

        g_params.relaxationFactor = 1.0f;
        g_params.shapeCollisionMargin = 0.04f;

        g_sceneLower = Vec3(-1.0f);
        g_sceneUpper = Vec3(1.0f);
        g_drawPoints = false;

        g_params.radius = radius*1.8f;
        g_params.collisionDistance = 0.005f;

        g_drawPoints = render_type & 1;
        g_drawCloth = (render_type & 2) >>1;
        g_drawSprings = false;

    }

    virtual void CenterCamera(void)
    {
        g_camPos = Vec3(cam_x, cam_y, cam_z);
        g_camAngle = Vec3(cam_angle_x, cam_angle_y, cam_angle_z);
        g_screenHeight = cam_height;
        g_screenWidth = cam_width;
    }
};
