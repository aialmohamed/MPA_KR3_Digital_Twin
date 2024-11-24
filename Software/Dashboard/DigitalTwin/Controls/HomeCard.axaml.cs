using Avalonia;
using Avalonia.Controls.Primitives;
using Avalonia.Data;

namespace DigitalTwin.Controls;

public class HomeCard : TemplatedControl
{
    public HomeCard()
    {
        
    }

    /// <summary>
    /// Defines the <see cref="CardHeight"/> property.
    /// 
    /// </summary>
    public static readonly DirectProperty< HomeCard,string> CardHeightProperty =
        AvaloniaProperty.RegisterDirect<HomeCard, string>(
            nameof(CardHeight),
             o => o.CardHeight,
             (o, v) => o.CardHeight = v,
             defaultBindingMode: BindingMode.TwoWay);

        // default height 200
        private string _cardHeight;
        public string CardHeight
        {
            get {return _cardHeight;}
            set {SetAndRaise(CardHeightProperty, ref _cardHeight,value);}
        }

        /// <summary>
        /// Defines the <see cref="CardIconString"/> property.
        /// </summary>
        public static readonly DirectProperty< HomeCard,string> CardIconStringProperty =
        AvaloniaProperty.RegisterDirect<HomeCard, string>(
            nameof(CardIconString),
             o => o.CardIconString,
             (o, v) => o.CardIconString = v,
             defaultBindingMode: BindingMode.TwoWay);
        
        // Defualt Icon to info icon
        private string _cardIconString ;
        /// <summary>
        /// Gets or sets the CardIconString property. please <see cref="https://phosphoricons.com/"/> 
        /// </summary>
        public string CardIconString
        {
            get {return _cardIconString;}
            set {SetAndRaise(CardIconStringProperty, ref _cardIconString,value);}
        }

        public static readonly DirectProperty< HomeCard,string> CardTitleProperty =
        AvaloniaProperty.RegisterDirect<HomeCard, string>(
            nameof(CardTitle),
             o => o.CardTitle,
             (o, v) => o.CardTitle = v,
             defaultBindingMode: BindingMode.TwoWay);
        
        private string _cardTitle;
        public string CardTitle
        {
            get {return _cardTitle;}
            set {SetAndRaise(CardTitleProperty, ref _cardTitle,value);}
        }

        public static readonly DirectProperty< HomeCard,string> CardContentProperty =
        AvaloniaProperty.RegisterDirect<HomeCard, string>(
            nameof(CardContent),
             o => o.CardContent,
             (o, v) => o.CardContent = v,
             defaultBindingMode: BindingMode.TwoWay);
        
        private string _cardContent;
        public string CardContent
        {
            get {return _cardContent;}
            set {SetAndRaise(CardContentProperty, ref _cardContent,value);}
        }

        public static readonly DirectProperty< HomeCard,string> CardWidthProperty =
        AvaloniaProperty.RegisterDirect<HomeCard, string>(
            nameof(CardWidth),
             o => o.CardWidth,
             (o, v) => o.CardWidth = v,
             defaultBindingMode: BindingMode.TwoWay);
        
        private string _cardWidth;
        public string CardWidth
        {
            get {return _cardWidth;}
            set {SetAndRaise(CardWidthProperty, ref _cardWidth,value);}
        }
}