using Avalonia;
using Avalonia.Controls.Primitives;
using Avalonia.Data;

namespace DigitalTwin.Controls;

public class PositionControl : TemplatedControl
{

    public PositionControl()
    {
        
    }
        /// <summary>
    /// Defines the <see cref="CardHeight"/> property.
    /// 
    /// </summary>
    public static readonly DirectProperty< PositionControl,string> CardHeightProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
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
        public static readonly DirectProperty< PositionControl,string> CardIconStringProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
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

        public static readonly DirectProperty< PositionControl,string> CardTitleProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
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

        public static readonly DirectProperty< PositionControl,string> CardWidthProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
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

        public static readonly DirectProperty< PositionControl,string> XCoordinateProperty  =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(XCoordinate),
             o => o.XCoordinate,
             (o, v) => o.XCoordinate = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _xCoordinate;
        public string XCoordinate
        {
            get {return _xCoordinate;}
            set {SetAndRaise(XCoordinateProperty, ref _xCoordinate,value);}
        }

        public static readonly DirectProperty< PositionControl,string> YCoordinateProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(YCoordinate),
             o => o.YCoordinate,
             (o, v) => o.YCoordinate = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _yCoordinate;
        public string YCoordinate
        {
            get {return _yCoordinate;}
            set {SetAndRaise(YCoordinateProperty, ref _yCoordinate,value);}
        }

        public static readonly DirectProperty< PositionControl,string> ZCoordinateProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(ZCoordinate),
             o => o.ZCoordinate,
             (o, v) => o.ZCoordinate = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _zCoordinate;
        public string ZCoordinate
        {
            get {return _zCoordinate;}
            set {SetAndRaise(ZCoordinateProperty, ref _zCoordinate,value);}
        }

        public static readonly DirectProperty< PositionControl,string> RollProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(Roll),
             o => o.Roll,
             (o, v) => o.Roll = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _roll;
        public string Roll
        {
            get {return _roll;}
            set {SetAndRaise(RollProperty, ref _roll,value);}
        }
        public static readonly DirectProperty< PositionControl,string> PitchProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(Pitch),
             o => o.Pitch,
             (o, v) => o.Pitch = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _pitch;
        public string Pitch
        {
            get {return _pitch;}
            set {SetAndRaise(PitchProperty, ref _pitch,value);}
        }

        public static readonly DirectProperty< PositionControl,string> YawProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(Yaw),
             o => o.Yaw,
             (o, v) => o.Yaw = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _yaw;
        public string Yaw
        {
            get {return _yaw;}
            set {SetAndRaise(YawProperty, ref _yaw,value);}
        }

        public static readonly DirectProperty< PositionControl,string> GripperProperty =
        AvaloniaProperty.RegisterDirect<PositionControl, string>(
            nameof(Gripper),
             o => o.Gripper,
             (o, v) => o.Gripper = v,
             defaultBindingMode: BindingMode.TwoWay);
        private string _gripper;
        public string Gripper
        {
            get {return _gripper;}
            set {SetAndRaise(GripperProperty, ref _gripper,value);}
        }

}