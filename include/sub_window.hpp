#pragma once

#include <cinder/app/AppNative.h>
#include <cinder/gl/gl.h>
#include <cinder/gl/Fbo.h>
#include <cinder/gl/Texture.h>
#include <cinder/gl/GlslProg.h>
#include <cinder/params/Params.h>

#include <CinderOpenCV.h>

namespace viz {

  /**
  * @struct SubWindow
  * @brief Sub window regions to draw to in the main viewer.
  * Allows the main window to be split up into different viewports.
  */
  class SubWindow {


  public:

    /**
    * Empty constructor.
    */
    SubWindow() : can_save_(false) { }

    /**
    * Create a window with dimensions.
    * @param[in] name The name of this sub window.
    * @param[in] start_x The x coordinate of the top left of the box in the main window reference frame.
    * @param[in] start_y The y coordinate of the top left of the box in the main window reference frame.
    * @param[in] width The height of the sub window in pixels.
    * @param[in] height The width of the sub window in pixels.
    */
    void Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, int draw_width, int draw_height, bool can_save);
    void Init(const std::string &name, int start_x, int start_y, int eye_width, int eye_height, bool can_save);

    void Bind();

    void BindAndClear();

    void UnBind();

    bool CanSave();

    bool IsSaving() const;

    void WriteFrameToFile();

    void Draw(ci::params::InterfaceGlRef params);

    void Draw(ci::params::InterfaceGlRef params, ci::Vec2i tl, ci::Vec2i size);

    void Draw();

    void InitSavingWindow();

    ci::Rectf GetRect() const { return window_coords_; }
    ci::Rectf GetRectWithBuffer() const { ci::Rectf r = window_coords_; r.scaleCentered(0.95); return r; }
    size_t Width() const { return window_coords_.getWidth(); }
    size_t Height() const { return window_coords_.getHeight(); }

    static std::string output_directory;

    protected:

    ci::Rectf window_coords_; /**< The window coordinates within the main window reference frame. */
    boost::shared_ptr<ci::gl::Fbo> framebuffer_; /**< The framebuffer of size width, height which is rendered to when we draw to this subwindow. */
    ci::gl::Texture texture_; /**< The texture that is attached to this framebuffer. */

    std::string name_;
    cv::VideoWriter writer_;

    bool can_save_;
    ci::params::InterfaceGlRef save_params_;

  };




}