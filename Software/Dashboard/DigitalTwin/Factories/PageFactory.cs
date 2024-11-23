

using System;
using DigitalTwin.Data;
using DigitalTwin.ViewModels;

namespace DigitalTwin.Factories;

public class PageFactory(Func<ApplicationPageNames, PageViewModel> factory)
{
    public PageViewModel GetPageViewModel(ApplicationPageNames pageName) => factory.Invoke(pageName);
}