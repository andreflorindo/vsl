// Inspired from moveit_setup_assistant, header_wighet.h, start_screen_widget.h, setup_screen_widget.h
#ifndef VSL_SCREEN_WINDOW_H_
#define VSL_SCREEN_WINDOW_H_

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

#include <urdf/model.h>                                       // for testing a valid urdf is loaded
#include <srdfdom/model.h>    // for testing a valid srdf is loaded


#include <moveit/setup_assistant/tools/moveit_config_data.h>  // common datastructure class



typedef struct Path
{
    std::vector<double> vector;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};



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
            void pathChanged(const QString& path);                                                      //<----Review
            void pathEditingFinished();                                                                   //<----Review

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
            SelectModeWidget* select_mode_;                                                                                  //<----Review
            QPushButton* btn_load_;
            QLabel* next_label_;
            QProgressBar* progress_bar_;

            moveit_setup_assistant::MoveItConfigDataPtr config_data_;                                                          //<----Review

        private Q_SLOTS:

            /// Button event for loading user chosen files
            void loadFilesClick();
        
            void readyToProgress();                                                                                         //<----Review

        private:

            /// Load exisiting package files
            bool loadExistingFiles();

            bool getFileContent(const std::string& srdf_file_path)

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
