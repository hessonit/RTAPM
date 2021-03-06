#include "viewer.h"
#include <cstdlib>


SimpleViewer::SimpleViewer() : shader_folder("src/shader/"), 
                   win_width(1366), //600
                   win_height(768) //400
{
    // init glfw - if already initialized nothing happens
    int init = glfwInit();
    offsetX = 0;
    offsetY = 0;
    rot = 0;
}

 static void glfwErrorCallback(int error, const char* description)
 {
   std::cerr << "GLFW error " << error << " " << description << std::endl;
 }

void SimpleViewer::setSize(int width, int height)
{
    win_width = width;
    win_height = height;
}

void SimpleViewer::initialize()
{
    GLFWerrorfun prev_func = glfwSetErrorCallback(glfwErrorCallback);
    if (prev_func)
      glfwSetErrorCallback(prev_func);

    // setup context
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
    //window =  glfwCreateWindow(win_width, win_height, "Viewer (press ESC to exit)", glfwGetPrimaryMonitor(), NULL);
     window = glfwCreateWindow(win_width, win_height, "Viewer (press ESC to exit)", 0, NULL); // 0 -> glfwGetPrimaryMonitor() (fullscreen)
    if (window == NULL)
    {
        std::cerr << "Failed to create opengl window." << std::endl;
        exit(-1);
    }

    glfwMakeContextCurrent(window);
    OpenGLBindings *b = new OpenGLBindings();
    flextInit(b);
    gl(b);

    std::string vertexshadersrc = ""
        "#version 330\n"
                                                
        "in vec2 Position;"
        "in vec2 TexCoord;"
                    
        "out VertexData{"
        "vec2 TexCoord;" 
        "} VertexOut;"  
                    
        "void main(void)"
        "{"
        "    gl_Position = vec4(Position, 0.0, 1.0);"
        "    VertexOut.TexCoord = TexCoord;"
        "}";
    std::string grayfragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "vec4 tempColor;"
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
        
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
            "ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"
            "tempColor = texelFetch(Data, uv);"
            "Color = vec4(tempColor.x/4500, tempColor.x/4500, tempColor.x/4500, 1);"
        "}";
    std::string fragmentshader = ""
        "#version 330\n"
        
        "uniform sampler2DRect Data;"
        
        "in VertexData{"
        "    vec2 TexCoord;"
        "} FragmentIn;"
       
        "layout(location = 0) out vec4 Color;"
        
        "void main(void)"
        "{"
        "    ivec2 uv = ivec2(FragmentIn.TexCoord.x, FragmentIn.TexCoord.y);"

        "    Color = texelFetch(Data, uv);"
        "}";

    renderShader.setVertexShader(vertexshadersrc);
    renderShader.setFragmentShader(fragmentshader);
    renderShader.build();

    renderGrayShader.setVertexShader(vertexshadersrc);
    renderGrayShader.setFragmentShader(grayfragmentshader);
    renderGrayShader.build();


    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, SimpleViewer::key_callbackstatic);
    glfwSetWindowSizeCallback(window, SimpleViewer::winsize_callbackstatic);

    shouldStop = false;
}

void SimpleViewer::winsize_callbackstatic(GLFWwindow* window, int w, int h)
{
    SimpleViewer* viewer = reinterpret_cast<SimpleViewer*>(glfwGetWindowUserPointer(window));
    viewer->winsize_callback(window, w, h);
}

void SimpleViewer::winsize_callback(GLFWwindow* window, int w, int h)
{
    win_width = w;// /2;
    win_height = h;// /2;
}

void SimpleViewer::key_callbackstatic(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    SimpleViewer* viewer = reinterpret_cast<SimpleViewer*>(glfwGetWindowUserPointer(window));
    viewer->key_callback(window, key, scancode, action, mods);
}

void SimpleViewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    const int offset = 2;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        shouldStop = true;
    else if (key == GLFW_KEY_D && action == GLFW_PRESS)
        offsetX -= offset;
    else if (key == GLFW_KEY_S && action == GLFW_PRESS)
        offsetY += offset;
    else if (key == GLFW_KEY_A && action == GLFW_PRESS)
        offsetX += offset;
    else if (key == GLFW_KEY_W && action == GLFW_PRESS)
        offsetY -= offset;
    else if (key == GLFW_KEY_R && action == GLFW_PRESS)
        rot -= offset;
    else if (key == GLFW_KEY_F && action == GLFW_PRESS)
        rot += offset;
}

void SimpleViewer::onOpenGLBindingsChanged(OpenGLBindings *b)
{
    renderShader.gl(b);
    renderGrayShader.gl(b);
    rgb.gl(b);
    ir.gl(b);
}

bool SimpleViewer::render()
{
    // wipe the drawing surface clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLint x = 0, y = 0;
    int fb_width, fb_width_half, fb_height, fb_height_half;

    std::map<std::string, libfreenect2::Frame*>::iterator iter;

    for (iter = frames.begin(); iter != frames.end(); ++iter)
    {
        libfreenect2::Frame* frame = iter->second;

        // Using the frame buffer size to account for screens where window.size != framebuffer.size, e.g. retina displays
        glfwGetFramebufferSize(window, &fb_width, &fb_height);
        fb_width_half = (fb_width + 1);// / 2;
        fb_height_half = (fb_height + 1);// / 2;

        glViewport(x, y, fb_width_half, fb_height_half);
        x += fb_width_half;
        if (x >= (fb_width - 1))
        {
            x = 0;
            y += fb_height_half;
        }

        float w = static_cast<float>(frame->width);
        float h = static_cast<float>(frame->height);

        Vertex bl = { -1.0f, -1.0f, 0.0f, 0.0f };
        Vertex br = { 1.0f, -1.0f, w, 0.0f }; 
        Vertex tl = { -1.0f, 1.0f, 0.0f, h };
        Vertex tr = { 1.0f, 1.0f, w, h };
        Vertex vertices[] = {
            bl, tl, tr, 
            tr, br, bl
        };

        gl()->glGenBuffers(1, &triangle_vbo);
        gl()->glGenVertexArrays(1, &triangle_vao);

        gl()->glBindVertexArray(triangle_vao);
        gl()->glBindBuffer(GL_ARRAY_BUFFER, triangle_vbo);
        gl()->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        GLint position_attr = renderShader.getAttributeLocation("Position");
        gl()->glVertexAttribPointer(position_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
        gl()->glEnableVertexAttribArray(position_attr);

        GLint texcoord_attr = renderShader.getAttributeLocation("TexCoord");
        gl()->glVertexAttribPointer(texcoord_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(float)));
        gl()->glEnableVertexAttribArray(texcoord_attr);


        if (iter->first == "RGB" || iter->first == "registered")
        {
            renderShader.use();

            rgb.allocate(frame->width, frame->height);
            std::copy(frame->data, frame->data + frame->width * frame->height * frame->bytes_per_pixel, rgb.data);
            rgb.flipY();
            rgb.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);

            rgb.deallocate();

        }
        else
        {
            renderGrayShader.use();

            ir.allocate(frame->width, frame->height);
            std::copy(frame->data, frame->data + frame->width * frame->height * frame->bytes_per_pixel, ir.data);
            ir.flipY();
            ir.upload();
            glDrawArrays(GL_TRIANGLES, 0, 6);
            ir.deallocate();
        }
    }

    // put the stuff we've been drawing onto the display
    glfwSwapBuffers(window);
    // update other events like input handling 
    glfwPollEvents();

    return shouldStop || glfwWindowShouldClose(window);
}

void SimpleViewer::addFrame(std::string id, libfreenect2::Frame* frame)
{
    frames[id] = frame;
}

void SimpleViewer::stopWindow()
{
    glfwWindowShouldClose(window);
}
