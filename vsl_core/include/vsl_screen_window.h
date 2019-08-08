// Inspired from setup_assistant_widget.h, header_wighet.h, start_screen_widget.h, setup_screen_widget.h
#ifndef VSL_SCREEN_WINDOW_H_
#define VSL_SCREEN_WINDOW_H_

// ROS
#include <ros/ros.h>

// C
#include <fstream>  
#include <streambuf>
#include <iostream>

// Qt
#include <QCloseEvent>
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
#include <QObject>
#include <QEvent>
#include <QListWidget>
#include <QSplitter>
#include <QStringList>
#include <QStackedLayout>
#include <QListWidgetItem>

// Boost
#include <boost/filesystem/path.hpp>        // for reading folders/files
#include <boost/filesystem/operations.hpp>  // for reading folders/files
#include <boost/program_options.hpp>  // for parsing input arguments
#include <boost/thread/mutex.hpp>

namespace vsl_screen_window
{
    struct Path
    {
        std::string filename;
        std::vector<double> vector;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;
    };
    
    class StartScreenWidget
    {
        Q_OBJECT
        
        public:
            StartScreenWidget(QWidget* parent);
            //~StartScreenWidget() override;                                                                                 //<----Review
            QPushButton* btn_load_;
            QLabel* next_label_;
            QProgressBar* progress_bar_;

        private Q_SLOTS:

            /// Button event for loading user chosen files
            void loadFilesClick();
        
            void readyToProgress();                                                                                         //<----Review

        private:

            /// Load exisiting package files
            bool loadExistingFiles();

            bool getFileContent(const std::string& srdf_file_path);

    };

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


}



#endif
