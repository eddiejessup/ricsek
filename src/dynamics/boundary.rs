use nalgebra::DefaultAllocator;

fn wrap1(x: f64, l: f64) -> f64 {
    if x < -l * 0.5 {
        // let n_wrap = (x / (l * 0.5)).abs().ceil();
        // if n_wrap > 1.0 {
        //   panic!("n_wrap = {} > 1", n_wrap);
        // }
        x + l
    } else if x > l * 0.5 {
        x - l
    } else {
        x
    }
}

pub fn wrap<D>(r: &mut nalgebra::OPoint<f64, D>, l: f64)
where
    D: nalgebra::DimName,
    DefaultAllocator: nalgebra::allocator::Allocator<f64, D>,
{
    r.apply(|x| *x = wrap1(*x, l));
}
