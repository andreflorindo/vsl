// Inspired from moveit_setup_assistant: header_wighet.cpp, start_screen_widget.cpp, setup_screen_widget.cpp


#include <vsl_sreen_window.h>


void SetupScreenWidget::focusGiven()
{}

bool SetupScreenWidget::focusLost()
{
    return true;  // accept switching by default
}


namespace vsl_screen_window
{
    namespace fs = boost::filesystem;

    HeaderWidget::HeaderWidget(const std::string& title, const std::string& instructions, QWidget* parent) : QWidget(parent)
    {
        // Basic widget container
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setAlignment(Qt::AlignTop);
        
        // Horizontal layout splitter
        QHBoxLayout* hlayout = new QHBoxLayout();

        // Page Title
        QLabel* page_title = new QLabel(this);
        page_title->setText(title.c_str());
        QFont page_title_font(QFont().defaultFamily(), 18, QFont::Bold);
        page_title->setFont(page_title_font);
        page_title->setWordWrap(true);
        layout->addWidget(page_title);
        layout->setAlignment(page_title, Qt::AlignTop);

        // Page Instructions
        QLabel* page_instructions = new QLabel(this);
        page_instructions->setText(instructions.c_str());
        page_instructions->setWordWrap(true);
        page_instructions->setMinimumWidth(1);
        layout->addWidget(page_instructions);
        layout->setAlignment(page_instructions, Qt::AlignTop);

        // Line Edit for path
        path_box_ = new QLineEdit(this);
        connect(path_box_, SIGNAL(textChanged(QString)), this, SIGNAL(pathChanged(QString)));   // <---- pathChanged
        connect(path_box_, SIGNAL(editingFinished()), this, SIGNAL(pathEditingFinished()));     // <---- pathEditingFinished
        hlayout->addWidget(path_box_);

        // Button
        QPushButton* browse_button = new QPushButton(this);
        browse_button->setText("Browse");
        connect(browse_button, SIGNAL(clicked()), this, SLOT(btnFileDialog()));                  // <---- btnFileDialog Done
        hlayout->addWidget(browse_button);

        // Margin on bottom
        layout->setContentsMargins(0, 0, 0, 0);  // last 15

        // Add horizontal layer to verticle layer
        layout->addLayout(hlayout);

        this->setLayout(layout);
        // this->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
    }

    void HeaderWidget::btnFileDialog()
    {
        QString start_path;

        start_path = path_box_->text();

        if (load_only_)
        {
            path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");
        }
        else
        {
            path = QFileDialog::getSaveFileName(this, "Create/Load File", start_path, "");
        }

        // check they did not press cancel
        if (!path.isNull())
        {
            path_box_->setText(path);
        }
    }


    QString HeaderWidget::getQPath() const
    {
        return path_box_->text();
    }


    std::string HeaderWidget::getPath() const
    {
        return getQPath().toStdString();
    }

    void HeaderWidget::setPath(const QString& path)
    {
        path_box_->setText(path);
    }   

    void HeaderWidget::setPath(const std::string& path)
    {
        path_box_->setText(QString(path.c_str()));
    }


    StartScreenWidget::StartScreenWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
    : SetupScreenWidget(parent), config_data_(config_data)
    {
        // Basic widget container
        QVBoxLayout* layout = new QVBoxLayout(this);

        // Top Label Area ---------------------------------------------------
        HeaderWidget* header = new HeaderWidget("VSL Deposition Motion Planner", "Specify the location of the file containing the laydown path to perform.", this);
        layout->addWidget(header);

        // Load settings box ---------------------------------------------
        QHBoxLayout* load_files_layout = new QHBoxLayout();
        progress_bar_ = new QProgressBar(this);
        progress_bar_->setMaximum(100);
        progress_bar_->setMinimum(0);
        progress_bar_->hide();
        load_files_layout->addWidget(progress_bar_);
        btn_load_ = new QPushButton("&Load Files", this);
        btn_load_->setMinimumWidth(180);
        btn_load_->setMinimumHeight(40);
        load_files_layout->addWidget(btn_load_);
        load_files_layout->setAlignment(btn_load_, Qt::AlignRight);
        connect(btn_load_, SIGNAL(clicked()), this, SLOT(loadFilesClick()));

        // Next step instructions
        next_label_ = new QLabel(this);
        QFont next_label_font(QFont().defaultFamily(), 11, QFont::Bold);
        next_label_->setFont(next_label_font);
        next_label_->setText("Success! Use the left navigation pane to continue.");
        next_label_->hide();  // only show once the files have been loaded.

        // Final Layout Setup ---------------------------------------------
        // Alignment
        layout->setAlignment(Qt::AlignTop);
        // Verticle Spacer
        QWidget* vspacer = new QWidget(this);
        vspacer->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        layout->addWidget(vspacer);
        // Attach bottom layout
        layout->addWidget(next_label_);
        layout->setAlignment(next_label_, Qt::AlignRight);
        layout->addLayout(load_files_layout);
        this->setLayout(layout);

        // Debug mode: auto load the configuration file by clicking button after a timeout
        if (config_data_->debug_)
        {
            // select_mode_->btn_exist_->click();
            QTimer* update_timer = new QTimer(this);
            update_timer->setSingleShot(true);  // only run once
            connect(update_timer, SIGNAL(timeout()), btn_load_, SLOT(click()));
            update_timer->start(100);
        }
    }

    void StartScreenWidget::loadFilesClick()
    {
        btn_load_->setDisabled(true);
        progress_bar_->show();

        bool result;

        result = loadExistingFiles();

        // Check if there was a failure loading files
        if (!result)
        {
            btn_load_->setDisabled(false);
            progress_bar_->hide();
        }
    }

    bool StartScreenWidget::loadPackageSettings(bool show_warnings)
    {
        // Get the package path
        std::string package_path_input = stack_path_->getPath();

        // Check that input is provided
        if (package_path_input.empty())
        {
            if (show_warnings)
            {
            QMessageBox::warning(this, "Error Loading Files", "Please specify a configuration package path to load.");
            }
            return false;
        }

        // check that the folder exists
        if (!config_data_->setPackagePath(package_path_input))
        {
            if (show_warnings)
            {
                QMessageBox::critical(this, "Error Loading Files", "The specified path is not a directory or is not accessable");
            }
            return false;
        }

        std::string setup_assistant_path;

        // Check if the old package is a setup assistant package. If it is not, quit
        if (!config_data_->getSetupAssistantYAMLPath(setup_assistant_path))
        {
            if (show_warnings)
            QMessageBox::warning(
                this, "Incorrect Directory/Package",
                QString("The chosen package location exists but was not created using MoveIt! Setup Assistant. "
                        "If this is a mistake, provide the missing file: ")
                    .append(setup_assistant_path.c_str()));
            return false;
        }

        // Get setup assistant data
        if (!config_data_->inputSetupAssistantYAML(setup_assistant_path))
        {
            if (show_warnings)
            {
            QMessageBox::warning(this, "Setup Assistant File Error",
                            QString("Unable to correctly parse the setup assistant configuration file: ")
                                .append(setup_assistant_path.c_str()));
            }
            return false;
        }
        return true;
    }





























}


bool getFileContent(std::string fileName, std::vector<double> & path)
{
	// Open the File
	std::ifstream path_file(fileName.c_str());
 
	// Check if object is valid
	if(!path_file)
	{
		std::cerr << "Program could not open the file: "<<fileName<<std::endl;
		return false;
	}
 
	std::string str;
	// Read the next line from File until it reaches the end.
	while (std::getline(path_file, str))
	{
		// Line contains string of length > 0 then save it in vector
		if(str.size() > 0)
			path.push_back(str);
	}
	//Close The File
	path_file.close();
	return true;
}
