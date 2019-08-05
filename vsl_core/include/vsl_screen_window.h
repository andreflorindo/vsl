// Inspired from moveit_setup_assistant: header_wighet.h, start_screen_widget.h, setup_screen_widget.h


// ROS
#include <ros/ros.h>
#include <ros/package.h>  // for getting file path for loadng images

// Qt
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QString>
#include <QApplication>
#include <QFont>
#include <QFileDialog>

// C
#include <fstream>  
#include <streambuf>
#include <iostream>

// Boost
#include <boost/filesystem/path.hpp>        // for reading folders/files
#include <boost/filesystem/operations.hpp>  // for reading folders/files

// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>



#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_SETUP_SCREEN_WIDGET_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_SETUP_SCREEN_WIDGET_
#ifndef MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_START_SCREEN_WIDGET_
#define MOVEIT_MOVEIT_SETUP_ASSISTANT_WIDGETS_START_SCREEN_WIDGET_
#ifndef MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_HEADER_WIDGET_
#define MOVEIT_ROS_MOVEIT_SETUP_ASSISTANT_WIDGETS_HEADER_WIDGET_
#ifndef Q_MOC_RUN
#include <urdf/model.h>                                       // for testing a valid urdf is loaded
#include <srdfdom/model.h>                                    // for testing a valid srdf is loaded
#include <moveit/setup_assistant/tools/moveit_config_data.h>  // common datastructure class
#endif

class SetupScreenWidget : public QWidget
{
    Q_OBJECT

    public:
        SetupScreenWidget(QWidget* parent) : QWidget(parent)
        {
        }

        /// function called when widget is activated, allows to update/initialize GUI
        virtual void focusGiven();

        /// function called when widget lost focus, allows to accept/reject changes and to reject switching (returning false)
        virtual bool focusLost();

        Q_SIGNALS:
            /// Event for when the current screen is in modal view. Essential disabled the left navigation
            void isModal(bool isModal);

            /// Event for telling rviz to highlight a link of the robot
            void highlightLink(const std::string& name, const QColor&);

            /// Event for telling rviz to highlight a group of the robot
            void highlightGroup(const std::string& name);

            /// Event for telling rviz to unhighlight all links of the robot
            void unhighlightAll();

};

namespace vsl_screen_window
{
    class HeaderWidget : public QWidget
    {
        Q_OBJECT
        
        private:
            // Stores the path qstring
            QLineEdit* path_box_;

        Q_SIGNALS:
            void pathChanged(const QString& path);
            void pathEditingFinished();

        private Q_SLOTS:
            /// Load the file dialog
            void btnFileDialog();

        public:
            /// Contructor
            HeaderWidget(const std::string& title, const std::string& instructions, QWidget* parent);
            
            /// Returns the file path in QString format
            QString getQPath() const;

            /// Returns the file path in std::string format
            std::string getPath() const;

            /// Set the path with QString
            void setPath(const QString& path);

            /// Set the path with std string
            void setPath(const std::string& path);
    };


    class StartScreenWidget : public SetupScreenWidget
    {
        Q_OBJECT
        
        public:
            StartScreenWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);
            ~StartScreenWidget() override;
            SelectModeWidget* select_mode_;
            QPushButton* btn_load_;
            QLabel* next_label_;
            QProgressBar* progress_bar_;
            QImage* right_image_;
            QLabel* right_image_label_;

            moveit_setup_assistant::MoveItConfigDataPtr config_data_;

        private Q_SLOTS:
            /// User has chosen to show new options
            void showNewOptions();

            /// User has chosen to show edit options
            void showExistingOptions();

            /// Button event for loading user chosen files
            void loadFilesClick();

            /// load package settings
            void onPackagePathChanged(const QString& path);

            /// enable xacro arguments
            void onUrdfPathChanged(const QString& path);

        Q_SIGNALS:
            void readyToProgress();
            void loadRviz();

        private:
            /// load package settings from .setup_assistant file
            bool loadPackageSettings(bool show_warnings);

            /// Load chosen files for creating new package
            bool loadNewFiles();

            /// Load exisiting package files
            bool loadExistingFiles();

            /// Load URDF File to Parameter Server
            bool loadURDFFile(const std::string& urdf_file_path, const std::string& xacro_args);

            /// Load SRDF File
            bool loadSRDFFile(const std::string& srdf_file_path);

            /// Put SRDF File on Parameter Server
            bool setSRDFFile(const std::string& srdf_string);

            //// Extract the package/stack name and relative path to urdf from an absolute path name
            bool extractPackageNameFromPath();

            /// Make the full URDF path using the loaded .setup_assistant data
            bool createFullURDFPath();

            /// Make the full SRDF path using the loaded .setup_assistant data
            bool createFullSRDFPath(const std::string& package_path);

            /// Loads sensors_3d yaml file
            bool load3DSensorsFile();
    };

    class SelectModeWidget : public QFrame
    {
        Q_OBJECT
        private:
        private Q_SLOTS:

        public:
            SelectModeWidget(QWidget* parent);

            // Load file button
            QPushButton* btn_new_;
            QPushButton* btn_exist_;
            QLabel* widget_instructions_;

    };

}



#endif
